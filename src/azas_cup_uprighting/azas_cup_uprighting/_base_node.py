"""MoveIt 기반 Pick 노드 베이스 클래스.

공통 기능:
  - MoveIt 초기화, plan 파라미터
  - RealSense 카메라 콜백 (color / depth / intrinsics)
  - YOLO 모델 로드
  - Hand-Eye 변환, pixel→base 좌표 변환
  - Home 이동 + home_xyz/home_ori 캐싱
  - Approach + 재검출 루틴
  - cv2 메인 루프 (freeze 화면, 키 입력, 자동 모드)

자식 노드는 주로 다음을 override / 구현:
  - detect_and_pick(frame)        — pick 시퀀스
  - _select_target(detections)    — 다음 픽 대상 선정
  - _draw_detections(frame)       — (optional) 시각화
  - on_ready()                    — Home 이후 추가 init (e.g. scan)
  - is_auto_ready()               — auto 모드 트리거 가능 조건
  - _handle_key_extra(key)        — 추가 키 (e.g. 's' for box scan)
"""

import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scipy.spatial.transform import Rotation

from sensor_msgs.msg import CameraInfo, Image

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from .onrobot import RG
from . import _config as cfg
from ._motion import get_ee_matrix, make_pose, plan_and_execute
from . import _perception as perc

try:
    from ultralytics import YOLO
except ImportError as e:
    raise ImportError("pip install ultralytics") from e


def _parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in {"1", "true", "yes", "y", "on"}


class BaseMoveItPickNode(Node):
    """MoveIt + RealSense + YOLO + RG2 그리퍼 통합 베이스."""

    NODE_NAME        = "yolo_pick_base"
    MOVEIT_NODE_NAME = "yolo_pick_base_py"
    WINDOW_NAME      = "YOLO Pick"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        log = self.get_logger()

        # ── 카메라 상태 ──
        self.color_image = None
        self.depth_image = None
        self.intrinsics  = None

        # ── 픽 상태 ──
        self.declare_parameter("auto_pick", False)
        self.declare_parameter("exit_after_pick", False)
        self.declare_parameter("skip_initial_home_move", False)
        self.declare_parameter("controller_action_name", "/dsr01/dsr_moveit_controller/follow_joint_trajectory")
        self.declare_parameter("controller_action_wait_sec", 60.0)
        self.picking = False
        self.home_xyz = None     # (x, y, z) [m] — initialize_home 에서 설정
        self.home_ori = None     # quat dict {x, y, z, w}
        self._auto_mode = _parse_bool(self.get_parameter("auto_pick").value)
        self._exit_after_pick = _parse_bool(self.get_parameter("exit_after_pick").value)
        self._skip_initial_home_move = _parse_bool(
            self.get_parameter("skip_initial_home_move").value
        )
        self._last_pick_time = 0.0
        self._auto_pick_attempt_started = False
        self._detections: list[dict] = []
        self._pick_snapshot_detections: list[dict] | None = None
        self._frozen_frame = None
        self._camera_paused = False
        self._camera_lock = threading.RLock()

        # ── Hand-Eye ──
        self.gripper2cam, calib_file = perc.load_hand_eye()
        log.info(f"Hand-Eye 로드: {calib_file}")

        # ── 그리퍼 ──
        self.gripper = RG(cfg.GRIPPER_NAME, cfg.TOOLCHARGER_IP, cfg.TOOLCHARGER_PORT)

        # ── MoveIt ──
        # 현장 실행은 카메라 확인이 먼저다. MoveItPy가 joint state/PlanningScene
        # 대기에서 막혀도 OpenCV 화면은 떠야 하므로 모션이 필요해지는 순간까지
        # 초기화를 미룬다.
        self.robot = None
        self.arm = None
        self.robot_model = None
        self.ompl_params = None
        self.pilz_params = None

        # ── YOLO ──
        self.declare_parameter("model_path", cfg.YOLO_MODEL_PATH)
        self.model_path = str(self.get_parameter("model_path").value).strip()
        self.model_path = os.path.expanduser(self.model_path)
        log.info(f"YOLO 모델 로드: {self.model_path}")
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"YOLO model_path does not exist: {self.model_path}. "
                "Set model_path:=... or AZAS_CUP_UPRIGHTING_MODEL_PATH."
            )
        self.yolo = YOLO(self.model_path)
        log.info("YOLO 모델 로드 완료")

        # ── 카메라 구독 ──
        self.create_subscription(CameraInfo, cfg.TOPIC_CAM_INFO,
                                 self._cam_info_cb, 10)
        self.create_subscription(Image, cfg.TOPIC_COLOR,
                                 self._color_cb, 10)
        self.create_subscription(Image, cfg.TOPIC_DEPTH,
                                 self._depth_cb, 10)

    # ════════════════════════════════════════════
    #  내부 헬퍼
    # ════════════════════════════════════════════
    def _make_plan_params(self, pipeline, planner_id, *,
                          vel: float, acc: float, time: float):
        p = PlanRequestParameters(self.robot)
        p.planning_pipeline = pipeline
        p.planner_id = planner_id
        p.max_velocity_scaling_factor = vel
        p.max_acceleration_scaling_factor = acc
        p.planning_time = time
        return p

    def _ensure_moveit(self) -> bool:
        if self.robot is not None:
            return True

        log = self.get_logger()
        log.info("MoveItPy 지연 초기화 중...")
        try:
            self.robot = MoveItPy(node_name=self.MOVEIT_NODE_NAME)
        except RuntimeError as exc:
            self.robot = None
            self.arm = None
            self.robot_model = None
            self.ompl_params = None
            self.pilz_params = None
            log.error(f"MoveItPy 초기화 실패: {exc}")
            return False
        self.arm = self.robot.get_planning_component(cfg.GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.ompl_params = self._make_plan_params(
            "ompl", "RRTConnect", vel=0.08, acc=0.05, time=3.0)
        self.pilz_params = self._make_plan_params(
            "pilz_industrial_motion_planner", "PTP", vel=0.06, acc=0.04, time=3.0)
        self.on_moveit_ready()
        log.info("MoveItPy 지연 초기화 완료")
        return True

    # ── 콜백 ──
    def _cam_info_cb(self, msg):
        with self._camera_lock:
            if self._camera_paused:
                return
            self.intrinsics = {
                "fx": msg.k[0], "fy": msg.k[4],
                "ppx": msg.k[2], "ppy": msg.k[5],
            }

    def _color_cb(self, msg):
        image = self._imgmsg_to_bgr(msg)
        with self._camera_lock:
            if self._camera_paused:
                return
            self.color_image = image

    def _depth_cb(self, msg):
        image = self._imgmsg_to_array(msg)
        with self._camera_lock:
            if self._camera_paused:
                return
            self.depth_image = image

    # cv_bridge는 NumPy 1.x ABI로 빌드되어 ~/.local의 NumPy 2.x와 충돌
    # (_ARRAY_API not found → segfault)하므로 직접 변환한다.
    @staticmethod
    def _imgmsg_to_array(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        dtype_by_encoding = {
            "8uc1": np.uint8,
            "mono8": np.uint8,
            "8uc3": np.uint8,
            "rgb8": np.uint8,
            "bgr8": np.uint8,
            "16uc1": np.uint16,
            "mono16": np.uint16,
            "32fc1": np.float32,
        }
        channels_by_encoding = {
            "8uc1": 1,
            "mono8": 1,
            "8uc3": 3,
            "rgb8": 3,
            "bgr8": 3,
            "16uc1": 1,
            "mono16": 1,
            "32fc1": 1,
        }
        if encoding not in dtype_by_encoding:
            raise ValueError(f"unsupported image encoding: {msg.encoding}")

        dtype = dtype_by_encoding[encoding]
        channels = channels_by_encoding[encoding]
        itemsize = np.dtype(dtype).itemsize
        row_values = msg.step // itemsize
        data = np.frombuffer(msg.data, dtype=dtype)
        if msg.is_bigendian != (data.dtype.byteorder == ">"):
            data = data.byteswap().view(data.dtype.newbyteorder())
        if channels == 1:
            image = data.reshape((msg.height, row_values))[:, : msg.width]
        else:
            image = data.reshape((msg.height, row_values // channels, channels))[:, : msg.width, :]
        return np.ascontiguousarray(image)

    @classmethod
    def _imgmsg_to_bgr(cls, msg: Image) -> np.ndarray:
        image = cls._imgmsg_to_array(msg)
        encoding = msg.encoding.lower()
        if encoding in {"bgr8", "8uc3"}:
            return image
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding in {"mono8", "8uc1"}:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"unsupported color encoding: {msg.encoding}")

    # ════════════════════════════════════════════
    #  Perception 래퍼
    # ════════════════════════════════════════════
    def transform_to_base(self, cam_xyz_m):
        if not self._ensure_moveit():
            return None
        return perc.transform_to_base(self.robot, self.gripper2cam, cam_xyz_m)

    def pixel_to_base(self, px, py):
        if not self._ensure_moveit():
            return None
        with self._camera_lock:
            depth_image = None if self.depth_image is None else self.depth_image.copy()
            intrinsics = None if self.intrinsics is None else dict(self.intrinsics)
        return perc.pixel_to_base(
            self.robot, self.gripper2cam,
            depth_image, intrinsics,
            px, py, self.get_logger())

    def run_yolo(self, frame, depth_image=None):
        if depth_image is None:
            with self._camera_lock:
                depth_image = None if self.depth_image is None else self.depth_image.copy()
        return perc.run_yolo(self.yolo, frame, depth_image)

    def current_camera_snapshot(self):
        with self._camera_lock:
            frame = None if self.color_image is None else self.color_image.copy()
            depth_image = None if self.depth_image is None else self.depth_image.copy()
            intrinsics = None if self.intrinsics is None else dict(self.intrinsics)
        return frame, depth_image, intrinsics

    def pause_camera_for_pick(
        self,
        frame: np.ndarray | None = None,
        *,
        depth_image=None,
        intrinsics=None,
        detections=None,
    ):
        """Freeze the frame/depth/intrinsics chosen for the active pick."""
        with self._camera_lock:
            if frame is None:
                if self.color_image is None:
                    return None
                frame = self.color_image

            frozen_frame = frame.copy()
            frozen_depth = (
                None if depth_image is None
                else depth_image.copy()
            )
            if frozen_depth is None and self.depth_image is not None:
                frozen_depth = self.depth_image.copy()
            frozen_intrinsics = (
                None if intrinsics is None
                else dict(intrinsics)
            )
            if frozen_intrinsics is None and self.intrinsics is not None:
                frozen_intrinsics = dict(self.intrinsics)

            self.color_image = frozen_frame.copy()
            self.depth_image = None if frozen_depth is None else frozen_depth.copy()
            self.intrinsics = None if frozen_intrinsics is None else dict(frozen_intrinsics)
            self._camera_paused = True
            self._frozen_frame = frozen_frame.copy()
            snapshot_detections = self._detections if detections is None else detections
            self._pick_snapshot_detections = [dict(d) for d in snapshot_detections]
            self._detections = [dict(d) for d in self._pick_snapshot_detections]
            return frozen_frame

    def resume_camera_after_pick(self):
        with self._camera_lock:
            self._camera_paused = False
            self._pick_snapshot_detections = None

    def pick_snapshot_detections(self) -> list[dict] | None:
        with self._camera_lock:
            if self._pick_snapshot_detections is None:
                return None
            return [dict(d) for d in self._pick_snapshot_detections]

    # ════════════════════════════════════════════
    #  Motion 래퍼
    # ════════════════════════════════════════════
    def plan_pose(self, x, y, z, ori, params=None) -> bool:
        if not self._ensure_moveit():
            return False
        return plan_and_execute(
            self.robot, self.arm, self.get_logger(),
            pose_goal=make_pose(x, y, z, ori),
            params=params or self.pilz_params,
            node=self,
            controller_action_name=self.get_parameter("controller_action_name").value,
            controller_action_wait_sec=float(
                self.get_parameter("controller_action_wait_sec").value
            ))

    def plan_state(self, state, params=None) -> bool:
        if not self._ensure_moveit():
            return False
        return plan_and_execute(
            self.robot, self.arm, self.get_logger(),
            state_goal=state,
            params=params or self.ompl_params,
            node=self,
            controller_action_name=self.get_parameter("controller_action_name").value,
            controller_action_wait_sec=float(
                self.get_parameter("controller_action_wait_sec").value
            ))

    def go_home_pose(self) -> bool:
        """관절 home 자세로 이동."""
        if not self._ensure_moveit():
            return False
        home_state = RobotState(self.robot_model)
        home_state.joint_positions = cfg.HOME_JOINTS
        home_state.update()
        return self.plan_state(home_state)

    # ════════════════════════════════════════════
    #  Approach + 재검출
    # ════════════════════════════════════════════
    def approach_and_redetect(self, target_cls_id: int, target_xy):
        """target XY 위로 EE 미세 이동 → 재검출 → 화면 중앙 가장 가까운 동일 클래스 detection.

        실패 시 None.
        """
        log = self.get_logger()
        if self._camera_paused:
            log.error("재검출 차단: pick 카메라 freeze 중입니다.")
            return None

        ori = self.home_ori
        ox, oy = cfg.APPROACH_OFFSET
        if not self._ensure_moveit():
            return None
        cur_ee = get_ee_matrix(self.robot)
        ax = target_xy[0] + ox
        ay = target_xy[1] + oy
        az = cur_ee[2, 3]

        log.info(
            f"[Approach] target_xy=({target_xy[0]:.3f}, {target_xy[1]:.3f}) "
            f"+ offset -> EE=({ax:.3f}, {ay:.3f}, {az:.3f})"
        )
        if not self.plan_pose(ax, ay, az, ori):
            log.error("Approach 실패")
            return None
        time.sleep(cfg.APPROACH_SETTLE)

        if self.color_image is None:
            log.error("재검출 프레임 없음")
            return None
        new_frame = self.color_image.copy()
        new_detections = self.run_yolo(new_frame)
        self._detections   = new_detections
        self._frozen_frame = new_frame.copy()

        same_cls = [d for d in new_detections if d["cls_id"] == target_cls_id]
        if not same_cls:
            log.error(f"재검출 실패: cls={target_cls_id} 없음")
            return None

        h, w = new_frame.shape[:2]
        cx_img, cy_img = w // 2, h // 2
        return min(
            same_cls,
            key=lambda d: (d["cx"] - cx_img) ** 2 + (d["cy"] - cy_img) ** 2,
        )

    # ════════════════════════════════════════════
    #  시각화 (기본 구현 — 자식이 override 가능)
    # ════════════════════════════════════════════
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """기본: 모든 detection 박스 + 다음 픽 대상은 녹색."""
        vis = frame.copy()
        next_target = (self._select_target(self._detections)
                       if self._detections else None)
        for det in self._detections:
            x1, y1, x2, y2 = det["box"]
            color = (0, 255, 0) if det is next_target else (255, 100, 0)
            label = f"{det['cls_name']} {det['conf']:.2f}"
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.drawMarker(vis, (det["cx"], det["cy"]), color,
                           cv2.MARKER_CROSS, 20, 2)
        self._draw_hud(vis)
        return vis

    def _draw_hud(self, vis: np.ndarray):
        """상단 HUD (mode, detections 수)."""
        mode_txt = "AUTO" if self._auto_mode else "MANUAL"
        mode_col = (0, 255, 255) if self._auto_mode else (200, 200, 200)
        cv2.putText(vis, f"[{mode_txt}] {self._key_help_str()}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_col, 2)
        cv2.putText(vis, f"detections: {len(self._detections)}",
                    (10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (180, 180, 180), 1)

    def _key_help_str(self) -> str:
        return "p:pick a:auto ESC:quit"

    # ════════════════════════════════════════════
    #  Pick 백그라운드 + freeze
    # ════════════════════════════════════════════
    def _pick_in_thread(
        self,
        frame: np.ndarray,
        *,
        depth_image=None,
        intrinsics=None,
        detections=None,
        auto_trigger: bool = False,
    ):
        if self.picking:
            return
        if auto_trigger and self._auto_pick_attempt_started:
            self.get_logger().warn("auto pick은 이미 1회 시작되었습니다. 재검출/재시도 없이 무시합니다.")
            return
        if auto_trigger:
            self._auto_pick_attempt_started = True

        frozen_frame = self.pause_camera_for_pick(
            frame,
            depth_image=depth_image,
            intrinsics=intrinsics,
            detections=detections,
        )
        if frozen_frame is None:
            self.get_logger().error("pick 시작 실패: 카메라 프레임 없음")
            return
        self.get_logger().info("pick 시작: 카메라 입력을 고정하고 완료 전까지 새 프레임을 무시합니다.")

        def _work():
            success = False
            try:
                success = bool(self.detect_and_pick(frozen_frame))
            finally:
                if not (self._exit_after_pick and auto_trigger):
                    self._frozen_frame = None
                    self.resume_camera_after_pick()

            if self._exit_after_pick and (success or auto_trigger):
                if success:
                    self.get_logger().info("exit_after_pick=true and pick completed; closing cup_uprighting node")
                else:
                    self.get_logger().error(
                        "auto pick failed; closing cup_uprighting node without re-detection/retry"
                    )
                rclpy.shutdown()

        threading.Thread(target=_work, daemon=True).start()

    # ════════════════════════════════════════════
    #  자식이 구현 / override 할 메서드 (hooks)
    # ════════════════════════════════════════════
    def detect_and_pick(self, frame: np.ndarray):
        raise NotImplementedError

    def _select_target(self, detections):
        raise NotImplementedError

    def on_ready(self):
        """Home 이동 완료 후 호출. 자식이 추가 init 가능 (e.g. scan_box)."""
        pass

    def on_moveit_ready(self):
        """MoveIt 지연 초기화 직후 호출. 자식이 planning scene 설정 가능."""
        pass

    def is_auto_ready(self) -> bool:
        """auto 모드 트리거 전제 조건."""
        return True

    def _handle_key_extra(self, key: int):
        """ESC, p, a 외 추가 키 처리. 자식 override (e.g. 's' for scan)."""
        pass

    # ════════════════════════════════════════════
    #  메인 루프
    # ════════════════════════════════════════════
    def initialize_home(self) -> bool:
        log = self.get_logger()
        if self._skip_initial_home_move:
            log.info("[Init] Home 이동/MoveIt 초기화 생략: 카메라 화면을 먼저 시작")
            self.gripper.open_gripper()
            time.sleep(1.0)
            return True

        log.info("[Init] Home 이동")
        if not self._ensure_moveit():
            return False
        if not self.go_home_pose():
            log.error("Home 실패")
            return False
        time.sleep(0.5)

        T = get_ee_matrix(self.robot)
        self.home_xyz = (T[0, 3], T[1, 3], T[2, 3])
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        self.home_ori = {"x": float(qx), "y": float(qy),
                         "z": float(qz), "w": float(qw)}
        log.info(f"[Init] Home = ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}) m")

        self.gripper.open_gripper()
        time.sleep(1.0)
        return True

    def run(self):
        log = self.get_logger()
        cv2.namedWindow(self.WINDOW_NAME)

        executor = MultiThreadedExecutor()
        executor.add_node(self)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        if not self.initialize_home():
            return
        self.on_ready()
        log.info(f"=== Ready === {self._key_help_str()}")

        while rclpy.ok():
            # ── Freeze 분기 (pick / scan 진행 중) ──
            if self._frozen_frame is not None:
                vis = self._draw_detections(self._frozen_frame)
                cv2.putText(vis, "[BUSY... CAMERA FROZEN]",
                            (10, 102), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 0, 255), 2)
                cv2.imshow(self.WINDOW_NAME, vis)
                key = cv2.waitKey(30) & 0xFF
                if key == 27:
                    break
                continue

            # ── Live 분기 ──
            frame, depth_image, intrinsics = self.current_camera_snapshot()
            if frame is None:
                time.sleep(0.01)
                continue

            self._detections = self.run_yolo(frame, depth_image)

            now = time.time()
            if (self._auto_mode
                    and not self.picking
                    and not self._auto_pick_attempt_started
                    and self.is_auto_ready()
                    and (now - self._last_pick_time) >= cfg.AUTO_PICK_INTERVAL):
                if self._select_target(self._detections) is not None:
                    self._last_pick_time = now
                    self._pick_in_thread(
                        frame,
                        depth_image=depth_image,
                        intrinsics=intrinsics,
                        detections=self._detections,
                        auto_trigger=True,
                    )
                    continue

            vis = self._draw_detections(frame)
            cv2.imshow(self.WINDOW_NAME, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord("p"):
                log.info("[KEY] manual pick")
                self._pick_in_thread(
                    frame,
                    depth_image=depth_image,
                    intrinsics=intrinsics,
                    detections=self._detections,
                )
            elif key == ord("a"):
                self._auto_mode = not self._auto_mode
                log.info(f"[KEY] auto {'ON' if self._auto_mode else 'OFF'}")
            else:
                self._handle_key_extra(key)

        cv2.destroyAllWindows()


def run_node(node_cls):
    """공통 main() 헬퍼."""
    rclpy.init()
    node = node_cls()
    try:
        node.run()
    finally:
        node.destroy_node()
        # exit_after_pick 경로 등에서 컨텍스트가 이미 닫혀 있으면 재호출 시
        # RuntimeError로 exit code 1이 되어 라우터가 실패로 오인한다.
        if rclpy.ok():
            rclpy.shutdown()
