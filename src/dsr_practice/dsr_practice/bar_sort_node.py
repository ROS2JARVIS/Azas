#!/usr/bin/env python3
"""
bar_sort_node.py  –  3 bars 크기순 자동 Pick & Place

click_pick_node.py 기반:
- 카메라로 bar 3개 자동 검출 (OpenCV contour)
- 길이(minAreaRect 장변) 순으로 정렬
- 미리 정의된 3개 위치에 작은→큰 순서로 배치
"""

import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from .onrobot import RG


# ═══════════════════════════════════════════
#  설정
# ═══════════════════════════════════════════
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK    = "link_6"

HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}

# 안전 작업 영역 (m, base_link)
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.40
SAFE_Y_MAX =  0.40
SAFE_Z_MIN =  0.25

# Pick/Place 파라미터 (m)
Z_OFFSET = 0.20   # click_pick_node와 동일
SAFE_Z   = 0.40

# Place 위치 : x=0 에 1열 (작은 → 큰, base_link)
PLACE_POSITIONS = [
    (0.0, -0.20, 0.20),  # 0 : 가장 작은 bar
    (0.0,  0.00, 0.20),  # 1 : 중간
    (0.0,  0.20, 0.20),  # 2 : 가장 큰 bar
]

# 시작 후 카메라 안정화를 위한 대기 시간 (초)
CAMERA_WARMUP_SEC = 3.0

# Bar 검출 파라미터
MIN_CONTOUR_AREA_PX = 100   # 픽셀 노이즈 컷

# Depth(mm) 기반 bar 분류
# (d_min, d_max, 이름, rank)  rank 0=가장 작음, 2=가장 큼
# LONG   : 308~311 → 305~315
# MEDIUM : 318~321 → 315~325
# SHORT  : 328~331 → 325~335
BAR_CLASSES = [
    (325, 335, "SHORT",  0),
    (315, 325, "MEDIUM", 1),
    (305, 315, "LONG",   2),
]


def classify_depth_mm(d_mm):
    """depth(mm) → (이름, rank) / 해당 없으면 (None, -1)"""
    for d_min, d_max, name, rank in BAR_CLASSES:
        if d_min <= d_mm < d_max:
            return name, rank
    return None, -1

# 그리퍼
GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502

DOWN_ORI = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}


# ═══════════════════════════════════════════
#  유틸
# ═══════════════════════════════════════════
def clamp_to_safe_workspace(x, y, z, logger):
    if x < SAFE_X_MIN:
        logger.warning(f"x={x:.3f} → {SAFE_X_MIN}")
        x = SAFE_X_MIN
    if y < SAFE_Y_MIN:
        logger.warning(f"y={y:.3f} → {SAFE_Y_MIN}")
        y = SAFE_Y_MIN
    elif y > SAFE_Y_MAX:
        logger.warning(f"y={y:.3f} → {SAFE_Y_MAX}")
        y = SAFE_Y_MAX
    if z < SAFE_Z_MIN:
        logger.warning(f"z={z:.3f} → {SAFE_Z_MIN}")
        z = SAFE_Z_MIN
    return x, y, z


def plan_and_execute(robot, arm, logger, pose_goal=None,
                     state_goal=None, params=None):
    arm.set_start_state_to_current_state()
    if pose_goal is not None:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)
    elif state_goal is not None:
        arm.set_goal_state(robot_state=state_goal)
    else:
        logger.error("pose/state 없음")
        return False

    plan_result = (arm.plan(parameters=params)
                   if params is not None else arm.plan())
    if not plan_result:
        logger.error("Planning 실패")
        return False

    robot.execute(group_name=GROUP_NAME,
                  robot_trajectory=plan_result.trajectory,
                  blocking=True)
    return True


def make_pose(x, y, z, ori=None):
    if ori is None:
        ori = DOWN_ORI
    p = PoseStamped()
    p.header.frame_id = BASE_FRAME
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    p.pose.orientation.x = ori["x"]
    p.pose.orientation.y = ori["y"]
    p.pose.orientation.z = ori["z"]
    p.pose.orientation.w = ori["w"]
    return p


def get_ee_matrix(moveit_robot):
    psm = moveit_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        T = scene.current_state.get_global_link_transform(EE_LINK)
    return np.asarray(T, dtype=float)


def detect_bars(color_img, logger):
    """
    bar 후보 검출: OTSU contour → 픽셀 장변(length).
    반환: [{'pixel':(cx,cy), 'length_px':..., 'rect':rect}, ...]
    """
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  kernel, iterations=1)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    bars = []
    for c in contours:
        if cv2.contourArea(c) < MIN_CONTOUR_AREA_PX:
            continue
        rect = cv2.minAreaRect(c)
        (cx, cy), (rw, rh), _ = rect
        bars.append({
            "pixel":     (int(cx), int(cy)),
            "length_px": float(max(rw, rh)),
            "rect":      rect,
        })

    logger.info(f"검출된 contour: {len(bars)} 개")
    return bars, thresh


# ═══════════════════════════════════════════
#  BarSortNode
# ═══════════════════════════════════════════
class BarSortNode(Node):
    def __init__(self):
        super().__init__("bar_sort_node")
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics  = None

        # Hand-Eye
        calib_file = (
            Path(get_package_share_directory("dsr_practice"))
            / "config" / "T_gripper2camera.npy"
        )
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0  # mm → m
        self.get_logger().info(f"Hand-Eye 로드: {calib_file}")

        # 그리퍼
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        # MoveIt
        self.get_logger().info("MoveItPy 초기화 중…")
        self.robot = MoveItPy(node_name="bar_sort_moveit_py")
        self.arm   = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.get_logger().info("MoveItPy 초기화 완료")

        # Plan 파라미터
        self.ompl_params = PlanRequestParameters(self.robot)
        self.ompl_params.planning_pipeline = "ompl"
        self.ompl_params.planner_id = "RRTConnect"
        self.ompl_params.max_velocity_scaling_factor = 0.2
        self.ompl_params.max_acceleration_scaling_factor = 0.1
        self.ompl_params.planning_time = 2.0

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.15
        self.pilz_params.max_acceleration_scaling_factor = 0.1
        self.pilz_params.planning_time = 2.0

        self.home_xyz = None
        self.home_ori = None

        # 구독
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            self._cam_info_cb, 10)
        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self._color_cb, 10)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_cb, 10)

    # ── 콜백 ──
    def _cam_info_cb(self, msg):
        self.intrinsics = {
            "fx": msg.k[0], "fy": msg.k[4],
            "ppx": msg.k[2], "ppy": msg.k[5],
        }

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # ── depth 샘플링 (5×5 median, mm) ──
    def sample_depth_mm(self, px, py):
        if self.depth_image is None:
            return None
        h, w = self.depth_image.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            return None
        x0, x1 = max(0, px - 2), min(w, px + 3)
        y0, y1 = max(0, py - 2), min(h, py + 3)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        z = float(np.median(valid))
        if self.depth_image.dtype != np.uint16:
            z *= 1000.0
        return z

    # ── 좌표 변환 ──
    def transform_to_base(self, cam_xyz_m):
        coord = np.append(np.array(cam_xyz_m, dtype=float), 1.0)
        base2ee = get_ee_matrix(self.robot)
        base2cam = base2ee @ self.gripper2cam
        return (base2cam @ coord)[:3]

    def pixel_to_base(self, px, py):
        """(px,py) → base 좌표 (m). 주변 5x5 median으로 depth 안정화."""
        if self.depth_image is None or self.intrinsics is None:
            return None
        h, w = self.depth_image.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            return None

        x0, x1 = max(0, px - 2), min(w, px + 3)
        y0, y1 = max(0, py - 2), min(h, py + 3)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        z_raw = float(np.median(valid))
        z_m = (z_raw / 1000.0
               if self.depth_image.dtype == np.uint16 else z_raw)

        fx, fy   = self.intrinsics["fx"],  self.intrinsics["fy"]
        ppx, ppy = self.intrinsics["ppx"], self.intrinsics["ppy"]
        cam_x = (px - ppx) * z_m / fx
        cam_y = (py - ppy) * z_m / fy
        return self.transform_to_base((cam_x, cam_y, z_m))

    # ── Pick & Place ──
    def pick_and_place(self, bx, by, bz, place_xyz):
        """
        1) 현재 z로 pick XY 이동
        2) pick_z (= bz + Z_OFFSET) 하강
        3) gripper close
        4) SAFE_Z 상승
        5) place XY 이동 (SAFE_Z)
        6) place_z 하강
        7) gripper open
        8) SAFE_Z 상승
        """
        log = self.get_logger()
        ori = self.home_ori or DOWN_ORI

        pick_z = bz + Z_OFFSET
        px, py, pz = place_xyz
        place_z = max(pz, pick_z)  # pick 높이 이상 보장

        log.info(
            f"Pick ({bx:.3f},{by:.3f},pick_z={pick_z:.3f}) → "
            f"Place ({px:.3f},{py:.3f},{place_z:.3f})"
        )

        cur_ee = get_ee_matrix(self.robot)
        cur_z = cur_ee[2, 3]

        # 0) gripper open
        self.gripper.open_gripper()
        time.sleep(0.5)

        # 1) pick XY
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(bx, by, cur_z, ori),
                                params=self.pilz_params):
            log.error("[1] plan 실패"); return False

        # 2) pick_z 하강
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(bx, by, pick_z, ori),
                                params=self.pilz_params):
            log.error("[2] plan 실패"); return False

        # 3) close
        log.info("[3] Gripper CLOSE")
        self.gripper.close_gripper()
        time.sleep(1.0)

        # 4) SAFE_Z 상승
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(bx, by, SAFE_Z, ori),
                                params=self.pilz_params):
            log.error("[4] plan 실패"); return False

        # 5) place XY (SAFE_Z)
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(px, py, SAFE_Z, ori),
                                params=self.pilz_params):
            log.error("[5] plan 실패"); return False

        # 6) place_z 하강
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(px, py, place_z, ori),
                                params=self.pilz_params):
            log.error("[6] plan 실패"); return False

        # 7) open
        log.info("[7] Gripper OPEN")
        self.gripper.open_gripper()
        time.sleep(1.0)

        # 8) SAFE_Z 상승
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(px, py, SAFE_Z, ori),
                         params=self.pilz_params)
        return True

    # ── 정렬 시퀀스 ──
    def sort_bars(self):
        log = self.get_logger()
        if (self.color_image is None or self.depth_image is None
                or self.intrinsics is None):
            log.error("이미지/내참 준비 안 됨"); return

        bars, thresh = detect_bars(self.color_image, log)
        if not bars:
            log.error("contour 없음 — 중단")
            cv2.imshow("BarSort_Thresh", thresh)
            cv2.waitKey(500)
            return

        # 각 contour 중심의 depth 로 SHORT / MEDIUM / LONG 분류
        # rank당 후보가 여럿이면 contour 면적(length_px) 큰 것 선택
        classified = {}   # rank -> target dict
        for b in bars:
            cx, cy = b["pixel"]
            d_mm = self.sample_depth_mm(cx, cy)
            if d_mm is None:
                continue
            name, rank = classify_depth_mm(d_mm)
            if rank < 0:
                log.info(f"skip: d={d_mm:.0f}mm 범위 밖 @ ({cx},{cy})")
                continue
            cand = {
                "pixel":     (cx, cy),
                "length_px": b["length_px"],
                "depth_mm":  d_mm,
                "name":      name,
                "rank":      rank,
            }
            if rank not in classified \
                    or cand["length_px"] > classified[rank]["length_px"]:
                classified[rank] = cand

        missing = [r for r in (0, 1, 2) if r not in classified]
        if missing:
            names = ["SHORT", "MEDIUM", "LONG"]
            log.error(f"분류 부족: 누락 {[names[r] for r in missing]}")
            return

        # 시각화
        vis = self.color_image.copy()
        colors = {0: (0, 255, 0), 1: (0, 200, 255), 2: (0, 0, 255)}
        for rank in (0, 1, 2):
            t = classified[rank]
            cx, cy = t["pixel"]
            col = colors[rank]
            cv2.circle(vis, (cx, cy), 10, col, 2)
            cv2.putText(vis,
                f"{t['name']} d={t['depth_mm']:.0f}",
                (cx + 12, cy), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, col, 2)
        cv2.imshow("BarSort", vis)
        cv2.waitKey(500)

        log.info("─── 분류 결과 ───")
        for rank in (0, 1, 2):
            t = classified[rank]
            log.info(
                f"[{rank}] {t['name']:6s} "
                f"d={t['depth_mm']:.0f}mm pixel={t['pixel']}"
            )

        # Pick & Place (SHORT(0) → MEDIUM(1) → LONG(2))
        for rank in (0, 1, 2):
            t = classified[rank]
            cx, cy = t["pixel"]
            base = self.pixel_to_base(cx, cy)
            if base is None:
                log.error(f"{t['name']} 좌표 변환 실패 — 중단")
                return
            bx, by, bz = float(base[0]), float(base[1]), float(base[2])
            log.info(
                f"═══ {t['name']} (d={t['depth_mm']:.0f}mm, "
                f"base z={bz:.3f}) ═══"
            )
            ok = self.pick_and_place(bx, by, bz, PLACE_POSITIONS[rank])
            if not ok:
                log.error(f"{t['name']} 실패 — 시퀀스 중단")
                return
            time.sleep(0.5)

        log.info("========== 정렬 완료 ==========")

    # ── 메인 ──
    def run(self):
        log = self.get_logger()
        window = "BarSort"
        cv2.namedWindow(window)

        # Home 이동
        log.info("[Init] Home 이동")
        home_state = RobotState(self.robot_model)
        home_state.joint_positions = HOME_JOINTS
        home_state.update()
        if not plan_and_execute(self.robot, self.arm, log,
                                state_goal=home_state,
                                params=self.ompl_params):
            log.error("Home 이동 실패 — 종료")
            return
        time.sleep(0.5)

        # Home pose 저장
        T = get_ee_matrix(self.robot)
        self.home_xyz = (T[0, 3], T[1, 3], T[2, 3])
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        self.home_ori = {"x": float(qx), "y": float(qy),
                         "z": float(qz), "w": float(qw)}
        log.info(f"[Init] Home = ({T[0,3]:.3f},{T[1,3]:.3f},{T[2,3]:.3f})")

        self.gripper.open_gripper()
        time.sleep(1.0)

        # 프레임 수신 대기
        log.info("프레임 수신 대기…")
        t0 = time.time()
        while rclpy.ok() and (self.color_image is None
                              or self.depth_image is None
                              or self.intrinsics is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 10.0:
                log.error("타임아웃"); return

        # 카메라 안정화 대기 (spin 계속 돌려 최신 프레임 수신)
        log.info(f"카메라 안정화 {CAMERA_WARMUP_SEC:.1f}초 대기…")
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < CAMERA_WARMUP_SEC:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.color_image is not None:
                cv2.imshow(window, self.color_image)
                cv2.waitKey(1)

        # 자동 실행
        log.info("정렬 시퀀스 시작")
        self.sort_bars()

        # 완료 후 이미지만 띄워두고 ESC 대기
        log.info("완료. ESC 로 종료")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.color_image is None:
                continue
            cv2.imshow(window, self.color_image)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = BarSortNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
