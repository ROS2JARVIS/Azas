
#!/usr/bin/env python3
"""
쓰러진 컵을 인식하고 보정된 오프셋으로 똑바로 세우는(Uprighting) 시나리오 노드.
"""

import time
import os
import cv2
import numpy as np

from geometry_msgs.msg import Pose

from . import _config as cfg
from ._base_node import BaseMoveItPickNode, RobotState, run_node
from ._perception import (
    analyze_cup_mouth,
    calculate_cup_orientation,
)
from ._motion import get_ee_matrix, get_gripper_pose_by_cup
from scipy.spatial.transform import Rotation as R


# =====================================================================
# 테스트 토글: 카메라와 욜로가 없어도 모션을 테스트하려면 True로 설정
USE_MOCK_VISION = os.getenv("YOLO_USE_MOCK_VISION", "0").lower() in {
    "1", "true", "yes", "on"
}
# =====================================================================

CUP_LENGTH_M = 0.12  
CUP_DIAMETER_M = 0.072 
CUP_RADIUS_M   = CUP_DIAMETER_M / 2.0 
OBSERVE_SAFE_Z_M = float(os.getenv("YOLO_OBSERVE_SAFE_Z_M", "0.55"))
OBSERVE_OFFSET_X_M = float(os.getenv("YOLO_OBSERVE_OFFSET_X_M", "-0.07"))
OBSERVE_OFFSET_Y_M = float(os.getenv("YOLO_OBSERVE_OFFSET_Y_M", "0.0"))
GRASP_BODY_OFFSET_M = float(os.getenv("YOLO_GRASP_BODY_OFFSET_M", "0.025"))
PLACE_RELEASE_CLEARANCE_M = float(os.getenv("YOLO_PLACE_RELEASE_CLEARANCE_M", "0.055"))
PLACE_PRE_RELEASE_CLEARANCE_M = float(os.getenv("YOLO_PLACE_PRE_RELEASE_CLEARANCE_M", "0.10"))
PLACE_DESCENT_STEP_M = float(os.getenv("YOLO_PLACE_DESCENT_STEP_M", "0.015"))
GRASP_HOLD_SEC = float(os.getenv("YOLO_GRASP_HOLD_SEC", "2.0"))
HOLD_AFTER_UPRIGHT = os.getenv("YOLO_HOLD_AFTER_UPRIGHT", "1").lower() in {
    "1", "true", "yes", "on"
}
UPRIGHT_HOLD_SEC = float(os.getenv("YOLO_UPRIGHT_HOLD_SEC", "3.0"))
RETURN_TO_OBSERVE_AFTER_RUN = os.getenv("YOLO_RETURN_TO_OBSERVE_AFTER_RUN", "1").lower() in {
    "1", "true", "yes", "on"
}
# 카메라는 그리퍼에 달려 있으므로 roll은 카메라 하향 자세를 유지해야 합니다.
# 파지 방향 보정은 roll/joint5가 아니라 yaw 평면에서만 처리합니다.
GRIPPER_CLOSE_AXIS_OFFSET_RAD = np.radians(
    float(os.getenv("YOLO_GRIPPER_CLOSE_AXIS_OFFSET_DEG", "-90.0"))
)
GRASP_ROLL_DEG = float(os.getenv("YOLO_GRASP_ROLL_DEG", "180.0"))
USE_MARKER_THETA = os.getenv("YOLO_USE_MARKER_THETA", "1").lower() in {
    "1", "true", "yes", "on"
}
GRASP_YAW_OFFSET_RAD = np.radians(float(os.getenv("YOLO_GRASP_YAW_OFFSET_DEG", "0.0")))
ENABLE_PRE_GRASP_REALIGN = os.getenv("YOLO_ENABLE_PRE_GRASP_REALIGN", "1").lower() in {
    "1", "true", "yes", "on"
}
ALIGN_MAX_ITERS = int(os.getenv("YOLO_ALIGN_MAX_ITERS", "6"))
ALIGN_SETTLE_SEC = float(os.getenv("YOLO_ALIGN_SETTLE_SEC", "0.35"))
ALIGN_XY_TOL_M = float(os.getenv("YOLO_ALIGN_XY_TOL_M", "0.012"))
ALIGN_MAX_STEP_M = float(os.getenv("YOLO_ALIGN_MAX_STEP_M", "0.025"))
ENABLE_MOUTH_UP_ALIGN = os.getenv("YOLO_ENABLE_MOUTH_UP_ALIGN", "1").lower() in {
    "1", "true", "yes", "on"
}
MOUTH_UP_TOL_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_TOL_DEG", "30.0")))
ENABLE_MOUTH_UP_LOCK = os.getenv("YOLO_ENABLE_MOUTH_UP_LOCK", "1").lower() in {
    "1", "true", "yes", "on"
}
MOUTH_UP_LOCK_TOL_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_LOCK_TOL_DEG", "20.0")))
MOUTH_UP_LOOSE_TOL_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_LOOSE_TOL_DEG", "70.0")))
MOUTH_UP_MAX_ITERS = int(os.getenv("YOLO_MOUTH_UP_MAX_ITERS", "8"))
MOUTH_UP_MAX_STEP_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_MAX_STEP_DEG", "8.0")))
MOUTH_UP_MIN_STEP_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_MIN_STEP_DEG", "0.8")))
MOUTH_UP_GAIN = float(os.getenv("YOLO_MOUTH_UP_GAIN", "0.40"))
MOUTH_UP_COARSE_THRESH_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_COARSE_THRESH_DEG", "45.0")))
MOUTH_UP_COARSE_MAX_STEP_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_COARSE_MAX_STEP_DEG", "18.0")))
MOUTH_UP_COARSE_GAIN = float(os.getenv("YOLO_MOUTH_UP_COARSE_GAIN", "0.55"))
MOUTH_UP_OPPOSITE_THRESH_RAD = np.radians(
    float(os.getenv("YOLO_MOUTH_UP_OPPOSITE_THRESH_DEG", "90.0"))
)
MOUTH_UP_OPPOSITE_MAX_STEP_RAD = np.radians(
    float(os.getenv("YOLO_MOUTH_UP_OPPOSITE_MAX_STEP_DEG", "95.0"))
)
MOUTH_UP_OPPOSITE_GAIN = float(os.getenv("YOLO_MOUTH_UP_OPPOSITE_GAIN", "0.80"))
MOUTH_UP_STABLE_COUNT = int(os.getenv("YOLO_MOUTH_UP_STABLE_COUNT", "1"))
MOUTH_UP_WORSEN_THRESH_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_WORSEN_THRESH_DEG", "3.0")))
MOUTH_UP_YAW_SIGN = float(os.getenv("YOLO_MOUTH_UP_YAW_SIGN", "1.0"))
MOUTH_UP_SAMPLE_COUNT = int(os.getenv("YOLO_MOUTH_UP_SAMPLE_COUNT", "3"))
MOUTH_UP_SAMPLE_DELAY_SEC = float(os.getenv("YOLO_MOUTH_UP_SAMPLE_DELAY_SEC", "0.08"))
MOUTH_UP_MAX_SAMPLE_SPREAD_RAD = np.radians(
    float(os.getenv("YOLO_MOUTH_UP_MAX_SAMPLE_SPREAD_DEG", "35.0"))
)
MOUTH_UP_PROBE_STEP_RAD = np.radians(float(os.getenv("YOLO_MOUTH_UP_PROBE_STEP_DEG", "5.0")))
GRASP_LEFT_TOL_RAD = np.radians(float(os.getenv("YOLO_GRASP_LEFT_TOL_DEG", "35.0")))
MARKER_EDGE_MARGIN_PX = int(os.getenv("YOLO_MARKER_EDGE_MARGIN_PX", "20"))
MARKER_EDGE_ALLOW_UP_TOL_RAD = np.radians(
    float(os.getenv("YOLO_MARKER_EDGE_ALLOW_UP_TOL_DEG", "25.0"))
)
ENSURE_VISIBLE_MAX_RETRIES = int(os.getenv("YOLO_ENSURE_VISIBLE_MAX_RETRIES", "3"))
GRASP_YAW_VERIFY_TOL_RAD = np.radians(float(os.getenv("YOLO_GRASP_YAW_VERIFY_TOL_DEG", "70.0")))


def _normalize_angle(theta):
    return (theta + np.pi) % (2.0 * np.pi) - np.pi


def _mouth_up_error(marker_theta):
    """카메라 화면 위쪽(^)을 0도로 본 입구 벡터 오차."""
    if marker_theta is None:
        return None
    screen_up_theta = -np.pi / 2.0
    return _normalize_angle(marker_theta - screen_up_theta)


def _circular_mean(angles):
    return float(np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles))))


def _choose_grasp_yaw(mouth_theta, mouth):
    """관측 시점에서 계산한 입구 방향을 그리퍼 yaw로 변환합니다."""
    return _normalize_angle(mouth_theta + GRASP_YAW_OFFSET_RAD)


class MockGripper:
    """가상 환경 테스트를 위해 실제 Modbus 통신을 우회하는 가짜 그리퍼 클래스"""
    def open_gripper(self):
        print("[Mock Gripper] 가상 그리퍼 열림 (110mm)")

    def close_gripper(self):
        print("[Mock Gripper] 가상 그리퍼 닫힘")

    def move_gripper(self, width, force=None):
        print(f"[Mock Gripper] 가상 그리퍼 너비 이동 -> {width/10.0}mm")


class YoloCupUprightingNode(BaseMoveItPickNode):
    NODE_NAME        = "yolo_cup_uprighting_node"
    MOVEIT_NODE_NAME = "yolo_cup_uprighting_py"
    WINDOW_NAME      = "Cup Uprighting"

    def _make_gripper(self):
        if USE_MOCK_VISION:
            self.get_logger().info("가상 모드: Mock Gripper를 활성화합니다.")
            return MockGripper()
        return super()._make_gripper()

    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        vis = super()._draw_detections(frame)
        target = self._select_target(self._detections)
        if target is None:
            return vis

        raw_theta = (
            np.radians(-116.91)
            if USE_MOCK_VISION
            else calculate_cup_orientation(self.depth_image, target["box"], frame)
        )
        mouth = analyze_cup_mouth(frame, target["box"], raw_theta)
        mouth_theta = (
            mouth["marker_theta"]
            if USE_MARKER_THETA and mouth["found"]
            else mouth["mouth_theta"]
        )
        up_error = _mouth_up_error(mouth["marker_theta"]) if mouth["found"] else None
        grasp_yaw = _choose_grasp_yaw(mouth_theta, mouth)
        close_axis_theta = grasp_yaw + GRIPPER_CLOSE_AXIS_OFFSET_RAD

        cx, cy = target["cx"], target["cy"]
        axis_len = max(50, int(target["size"] * 0.45))
        grasp_len = max(35, int(target["size"] * 0.28))
        raw_end = (
            int(cx + axis_len * np.cos(raw_theta)),
            int(cy + axis_len * np.sin(raw_theta)),
        )
        mouth_end = (
            int(cx + axis_len * np.cos(mouth_theta)),
            int(cy + axis_len * np.sin(mouth_theta)),
        )
        grasp_end = (
            int(cx + grasp_len * np.cos(close_axis_theta)),
            int(cy + grasp_len * np.sin(close_axis_theta)),
        )

        cv2.arrowedLine(vis, (cx, cy), raw_end, (0, 220, 0), 2, tipLength=0.25)
        cv2.arrowedLine(vis, (cx, cy), mouth_end, (0, 220, 255), 3, tipLength=0.25)
        cv2.arrowedLine(vis, (cx, cy), grasp_end, (255, 180, 0), 3, tipLength=0.25)

        if mouth["found"]:
            sticker = mouth["sticker_center"]
            cv2.circle(vis, sticker, 7, (0, 0, 255), -1)
            mouth_state = "TOP OK" if mouth["is_top"] else "FLIPPED -> ROTATE 180"
            mouth_color = (0, 255, 0) if mouth["is_top"] else (0, 180, 255)
        else:
            mouth_state = "MOUTH UNKNOWN"
            mouth_color = (0, 0, 255)

        x1, y1, _, y2 = target["box"]
        text_y = max(82, y1 - 36)
        cv2.putText(
            vis,
            f"theta: {np.degrees(raw_theta):.1f} deg",
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 220, 0),
            2,
        )
        cv2.putText(
            vis,
            f"mouth: {mouth_state} {'MARKER' if USE_MARKER_THETA and mouth['found'] else 'AXIS'}",
            (x1, text_y + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            mouth_color,
            2,
        )
        cv2.putText(
            vis,
            f"grasp yaw: {np.degrees(grasp_yaw):.1f} deg",
            (x1, text_y + 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 180, 0),
            2,
        )
        if up_error is not None:
            up_ok = abs(up_error) <= MOUTH_UP_TOL_RAD
            cv2.putText(
                vis,
                f"up err: {np.degrees(up_error):+.1f} deg {'OK' if up_ok else 'ALIGN'}",
                (x1, text_y + 66),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 0) if up_ok else (0, 180, 255),
                2,
            )
        cv2.putText(
            vis,
            "green=axis yellow=mouth/red blue=grasp red=marker",
            (10, min(vis.shape[0] - 12, y2 + 28)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (230, 230, 230),
            1,
        )
        return vis

    
    def _select_target(self, detections):
        """
        현재 YOLO 모델의 실제 클래스 이름('cup')을 찾아 신뢰도가 가장 높은 객체를 선택
        """
        if not detections:
            return None
            
        target_classes = {"cup", "toppled_cup"}
        target_candidates = [d for d in detections if d["cls_name"] in target_classes]

        if not target_candidates:
            return None

        return max(target_candidates, key=lambda d: d["conf"])


    def run_yolo(self, frame):
        """MOCK 모드일 경우 가상의 쓰러진 컵 데이터를 반환, 아니면 부모(진짜 YOLO) 호출"""
        if USE_MOCK_VISION:
            return [{
                "cx": 320, "cy": 240, "conf": 0.95,
                "cls_id": 99, "cls_name": "toppled_cup",
                "box": (200, 150, 440, 330), # 가로로 누워있는 가상의 바운딩 박스
                "size": 240, "depth": 0.5
            }]
        else:
            return super().run_yolo(frame)

    def pixel_to_base(self, px, py):
        """MOCK 모드일 경우 가상의 3D 공간 좌표 반환, 아니면 진짜 카메라 Depth 매핑 호출"""
        if USE_MOCK_VISION:
        
            # 실제 추출된 X: 0.487m, Y: -0.022m
            # 역산된 컵 표면 Z: 0.051m
            return (0.487, -0.022, 0.051)
        else:
            return super().pixel_to_base(px, py)

    def _return_to_observe_pose(self):
        """시퀀스 종료 후 시작 관측 자세(HOME_JOINTS)로 복귀합니다."""
        log = self.get_logger()
        if not RETURN_TO_OBSERVE_AFTER_RUN:
            return
        if self.robot is None:
            return

        log.info("[Return] observe/home 관측 자세로 복귀합니다.")
        if not self.go_home_pose():
            log.error("[Return] observe/home 관측 자세 복귀 실패. 수동 확인이 필요합니다.")
            return
        self.gripper.open_gripper()
        log.info("[Return] observe/home 관측 자세 복귀 완료.")

    def _estimate_target_from_frame(self, frame):
        detections = self.run_yolo(frame)
        self._detections = detections
        target = self._select_target(detections)
        if target is None:
            return None

        base = self.pixel_to_base(target["cx"], target["cy"])
        if base is None:
            return None

        if USE_MOCK_VISION:
            raw_theta = np.radians(-116.91)
        else:
            raw_theta = calculate_cup_orientation(self.depth_image, target["box"], frame)

        mouth = analyze_cup_mouth(frame, target["box"], raw_theta)
        if USE_MARKER_THETA and mouth["found"]:
            mouth_theta = mouth["marker_theta"]
        elif not mouth["is_top"]:
            mouth_theta = mouth["mouth_theta"]
        else:
            mouth_theta = raw_theta

        grasp_yaw = _choose_grasp_yaw(mouth_theta, mouth)
        return {
            "target": target,
            "base": base,
            "mouth_theta": mouth_theta,
            "grasp_yaw": grasp_yaw,
            "mouth": mouth,
            "up_error": _mouth_up_error(mouth["marker_theta"]) if mouth["found"] else None,
        }

    def _estimate_mouth_from_frame(self, frame):
        """각도 정렬 전용 추정. depth/base 좌표가 없어도 빨간 점 방향만 확인합니다."""
        detections = self.run_yolo(frame)
        self._detections = detections
        target = self._select_target(detections)
        if target is None:
            return None

        if USE_MOCK_VISION:
            raw_theta = np.radians(-116.91)
        else:
            raw_theta = calculate_cup_orientation(self.depth_image, target["box"], frame)

        mouth = analyze_cup_mouth(frame, target["box"], raw_theta)
        mouth_theta = (
            mouth["marker_theta"]
            if USE_MARKER_THETA and mouth["found"]
            else mouth["mouth_theta"]
        )
        return {
            "target": target,
            "mouth_theta": mouth_theta,
            "mouth": mouth,
            "up_error": _mouth_up_error(mouth["marker_theta"]) if mouth["found"] else None,
        }

    def _grasp_left_error(self, grasp_yaw):
        close_axis_theta = _normalize_angle(grasp_yaw + GRIPPER_CLOSE_AXIS_OFFSET_RAD)
        return _normalize_angle(close_axis_theta - np.pi)

    def _initial_observe_errors(self, mouth, grasp_yaw):
        """초기 observe 화면의 입구/파지 방향 오차를 기록용으로 계산."""
        mouth_theta = mouth["marker_theta"] if mouth["found"] else mouth.get("mouth_theta")
        if mouth_theta is None:
            return None, None

        up_error = _mouth_up_error(mouth_theta)
        grasp_left_error = self._grasp_left_error(grasp_yaw)
        return up_error, grasp_left_error

    def detect_and_pick(self, frame: np.ndarray):
        log = self.get_logger()
        if self.picking:
            log.warn("이미 시퀀스 실행 중입니다.")
            return

        detections = self.run_yolo(frame)
        self._detections = detections
        target = self._select_target(detections)
        
        if target is None:
            log.warn("쓰러진 컵을 찾을 수 없습니다.")
            return False

        base = self.pixel_to_base(target["cx"], target["cy"])
        if base is None:
            log.error("픽셀 -> 베이스 3D 좌표 변환 실패.")
            return False
        bx, by, bz = base
        
        # 각도 추출
        if USE_MOCK_VISION:
            cup_theta = np.radians(-116.91)
        else:
            cup_theta = calculate_cup_orientation(self.depth_image, target["box"], frame)

       
        # ==========================================================
        
        mouth = analyze_cup_mouth(frame, target["box"], cup_theta)

        if USE_MARKER_THETA and mouth["found"]:
            cup_theta = mouth["marker_theta"]
            log.info("[VISION] 빨간 점 방향을 입구 방향으로 직접 사용합니다.")
        elif not mouth["is_top"]:
            log.info("[VISION] 컵이 반대로 누워있습니다. 카메라 상향 유지를 위해 파지 방향을 180도 뒤집습니다.")
            cup_theta = mouth["mouth_theta"]
        else:
            log.info("[VISION] 컵이 정방향입니다. 기본 파지 방향을 유지합니다.")

        grasp_yaw = _choose_grasp_yaw(cup_theta, mouth)
        log.info(f"[VISION] 관측 기준 파지 yaw: {np.degrees(grasp_yaw):.1f}도")
        initial_up_error, initial_grasp_left_error = self._initial_observe_errors(
            mouth, grasp_yaw
        )
        if initial_up_error is not None and initial_grasp_left_error is not None:
            log.info(
                "[VISION] 초기 observe 입구 벡터를 기록합니다. "
                f"up_err={np.degrees(initial_up_error):+.1f}도, "
                f"grasp_left_err={np.degrees(initial_grasp_left_error):+.1f}도. "
                "초기 12시 조건만으로 yaw 정렬을 생략하지 않습니다."
            )
        # ==========================================================

        self.picking = True
        success = False
        try:
            success = bool(self._pick_and_straighten(
                bx, by, bz, cup_theta, grasp_yaw,
            ))
        finally:
            self._return_to_observe_pose()
            self.picking = False
        return success

       


    def _observe_xy_from_cup(self, bx, by):
        observe_x = float(np.clip(
            bx + OBSERVE_OFFSET_X_M,
            cfg.SAFE_X_MIN,
            cfg.SAFE_X_MAX,
        ))
        observe_y = float(np.clip(
            by + OBSERVE_OFFSET_Y_M,
            cfg.SAFE_Y_MIN,
            cfg.SAFE_Y_MAX,
        ))
        return observe_x, observe_y

    def _body_grasp_xy_from_cup(self, bx, by, mouth_yaw):
        """입구 반대쪽으로 최종 파지점을 이동해 컵 몸통을 잡도록 보정."""
        offset = max(0.0, GRASP_BODY_OFFSET_M)
        if offset <= 1e-6:
            return bx, by
        grasp_x = float(np.clip(
            bx - offset * np.cos(mouth_yaw),
            cfg.SAFE_X_MIN,
            cfg.SAFE_X_MAX,
        ))
        grasp_y = float(np.clip(
            by - offset * np.sin(mouth_yaw),
            cfg.SAFE_Y_MIN,
            cfg.SAFE_Y_MAX,
        ))
        return grasp_x, grasp_y

    def _align_above_cup(self, bx, by, cup_theta, grasp_yaw, safe_z, observe_ori):
        log = self.get_logger()
        last_cup_bx, last_cup_by = bx, by
        last_obs_bx, last_obs_by = self._observe_xy_from_cup(bx, by)

        for i in range(ALIGN_MAX_ITERS):
            if self.color_image is None:
                log.warn("[Align] 카메라 프레임 없음. 현재 추정값으로 진행합니다.")
                break

            estimate = self._estimate_target_from_frame(self.color_image.copy())
            if estimate is None:
                log.warn("[Align] 재검출 실패. 현재 추정값으로 진행합니다.")
                break

            nbx, nby, _ = estimate["base"]
            cup_theta = estimate["mouth_theta"]
            grasp_yaw = estimate["grasp_yaw"]
            target_obs_bx, target_obs_by = self._observe_xy_from_cup(nbx, nby)
            dx = target_obs_bx - last_obs_bx
            dy = target_obs_by - last_obs_by
            xy_err = float(np.hypot(dx, dy))
            if xy_err > ALIGN_MAX_STEP_M:
                ratio = ALIGN_MAX_STEP_M / xy_err
                move_bx = last_obs_bx + dx * ratio
                move_by = last_obs_by + dy * ratio
            else:
                move_bx, move_by = target_obs_bx, target_obs_by

            log.info(
                f"[Align {i + 1}/{ALIGN_MAX_ITERS}] "
                f"cup=({nbx:.3f}, {nby:.3f}) -> "
                f"observe=({target_obs_bx:.3f}, {target_obs_by:.3f}), "
                f"xy_err={xy_err:.3f}m, "
                f"step=({move_bx - last_obs_bx:+.3f}, {move_by - last_obs_by:+.3f})m"
            )

            if not self.plan_pose(move_bx, move_by, safe_z, observe_ori):
                log.error("[Align] 카메라 하향 관측 자세 이동 실패. 시퀀스를 중단합니다.")
                return None

            last_cup_bx, last_cup_by = nbx, nby
            last_obs_bx, last_obs_by = move_bx, move_by
            time.sleep(ALIGN_SETTLE_SEC)

            if xy_err <= ALIGN_XY_TOL_M:
                log.info("[Align] 카메라 하향 자세에서 위치 정렬 완료.")
                return nbx, nby, cup_theta, grasp_yaw

        log.warn("[Align] 허용 오차 안에 완전히 들어오지 않았지만 마지막 관측값으로 진행합니다.")
        return last_cup_bx, last_cup_by, cup_theta, grasp_yaw

    def _ensure_cup_visible(self, bx, by, safe_z, observe_ori,
                             max_retries=3):
        """하강 전 컵 가시성 최종 확인.

        컵이 카메라 화면에 보이지 않으면 관측 자세(카메라 하향)로 복귀해 재탐색.
        성공 시 최신 estimate 반환, 실패 시 None.
        """
        log = self.get_logger()

        for attempt in range(max_retries):
            if self.color_image is None:
                log.warn(
                    f"[EnsureVisible {attempt + 1}/{max_retries}] "
                    "카메라 프레임 없음. 잠시 대기합니다."
                )
                time.sleep(0.3)
                continue

            estimate = self._estimate_target_from_frame(self.color_image.copy())
            if estimate is not None:
                nbx, nby, _ = estimate["base"]
                log.info(
                    f"[EnsureVisible {attempt + 1}/{max_retries}] "
                    f"컵 확인됨 ({nbx:.3f}, {nby:.3f})"
                )
                return estimate

            log.warn(
                f"[EnsureVisible {attempt + 1}/{max_retries}] "
                "컵이 화면에서 벗어났습니다. "
                "컵 뒤쪽 관측 위치로 복귀 후 재탐색합니다."
            )
            observe_bx, observe_by = self._observe_xy_from_cup(bx, by)
            if not self.plan_pose(observe_bx, observe_by, safe_z, observe_ori):
                log.error("[EnsureVisible] 관측 자세 이동 실패. 시퀀스를 중단합니다.")
                return None
            time.sleep(ALIGN_SETTLE_SEC)

        log.error(
            f"[EnsureVisible] {max_retries}회 재시도 후에도 컵을 "
            "찾지 못했습니다. 시퀀스를 중단합니다."
        )
        return None

    def _find_nearest_feasible_yaw(self, bx, by, safe_z, yaw,
                                    scan_range_deg=30.0, scan_step_deg=5.0):
        """이동 전 IK 가능한 가장 가까운 yaw를 탐색합니다. 없으면 None."""
        bx_c = float(np.clip(bx, cfg.SAFE_X_MIN, cfg.SAFE_X_MAX))
        by_c = float(np.clip(by, cfg.SAFE_Y_MIN, cfg.SAFE_Y_MAX))
        sz_c = float(np.clip(safe_z, cfg.SAFE_Z_MIN, cfg.SAFE_Z_MAX))

        def _feasible(candidate_yaw):
            ori = get_gripper_pose_by_cup(candidate_yaw, roll_deg=GRASP_ROLL_DEG)
            p = Pose()
            p.position.x, p.position.y, p.position.z = bx_c, by_c, sz_c
            p.orientation.x = ori["x"]
            p.orientation.y = ori["y"]
            p.orientation.z = ori["z"]
            p.orientation.w = ori["w"]
            state = RobotState(self.robot_model)
            return state.set_from_ik(cfg.GROUP_NAME, p, cfg.EE_LINK, 0.05)

        if _feasible(yaw):
            return yaw

        steps = int(scan_range_deg / scan_step_deg)
        for k in range(1, steps + 1):
            delta = np.radians(k * scan_step_deg)
            for sign in (+1, -1):
                candidate = _normalize_angle(yaw + sign * delta)
                if _feasible(candidate):
                    return candidate

        return None

    def _measure_mouth_up_error(self):
        """빨간 점 방향을 여러 프레임 평균내어 입구-12시 오차를 안정적으로 측정."""
        log = self.get_logger()
        errors = []
        axis_errors = []
        last_estimate = None
        sample_goal = max(1, MOUTH_UP_SAMPLE_COUNT)

        for _ in range(sample_goal):
            if self.color_image is None:
                break

            estimate = self._estimate_mouth_from_frame(self.color_image.copy())
            if estimate is not None:
                mouth = estimate["mouth"]
                up_error = estimate["up_error"]
                if up_error is not None and mouth["found"]:
                    sx, sy = mouth["sticker_center"]
                    x1, y1, x2, y2 = estimate["target"]["box"]
                    margin = MARKER_EDGE_MARGIN_PX
                    if (
                        sx <= x1 + margin or sx >= x2 - margin
                        or sy <= y1 + margin or sy >= y2 - margin
                    ):
                        if abs(up_error) <= MARKER_EDGE_ALLOW_UP_TOL_RAD:
                            log.warn(
                                "[MouthAlign] 빨간 점이 bbox 가장자리에 있지만 "
                                f"입구 오차가 {np.degrees(up_error):+.1f}도로 작아 "
                                "유효 샘플로 사용합니다."
                            )
                        else:
                            log.warn(
                                "[MouthAlign] 빨간 점이 bbox 가장자리에 있고 "
                                f"입구 오차도 {np.degrees(up_error):+.1f}도로 큽니다. "
                                "이 샘플은 사용하지 않습니다."
                            )
                            continue
                    errors.append(float(up_error))
                    last_estimate = estimate
                elif estimate.get("mouth_theta") is not None:
                    axis_errors.append(float(_mouth_up_error(estimate["mouth_theta"])))
                    last_estimate = estimate

            time.sleep(max(0.0, MOUTH_UP_SAMPLE_DELAY_SEC))

        if not errors:
            if not axis_errors:
                log.warn("[MouthAlign] 빨간 점/축 방향 샘플을 모두 얻지 못했습니다.")
                return None
            mean_error = _circular_mean(np.asarray(axis_errors, dtype=float))
            spread = max(abs(_normalize_angle(e - mean_error)) for e in axis_errors)
            log.warn(
                "[MouthAlign] 빨간 점 샘플을 얻지 못해 노란 축 벡터로 보정을 시작합니다. "
                f"axis_up_err={np.degrees(mean_error):+.1f}도, "
                f"spread={np.degrees(spread):.1f}도"
            )
            return {
                "up_error": mean_error,
                "sample_count": len(axis_errors),
                "spread": spread,
                "estimate": last_estimate,
                "source": "axis",
            }

        mean_error = _circular_mean(np.asarray(errors, dtype=float))
        spread = max(abs(_normalize_angle(e - mean_error)) for e in errors)
        if len(errors) >= 2 and spread > MOUTH_UP_MAX_SAMPLE_SPREAD_RAD:
            log.warn(
                f"[MouthAlign] 빨간 점 방향 샘플이 불안정합니다 "
                f"(spread={np.degrees(spread):.1f}도). 평균값으로 계속 진행합니다."
            )

        return {
            "up_error": mean_error,
            "sample_count": len(errors),
            "spread": spread,
            "estimate": last_estimate,
            "source": "marker",
        }

    def _move_to_yaw(self, bx, by, safe_z, yaw):
        feasible_yaw = self._find_nearest_feasible_yaw(bx, by, safe_z, yaw)
        if feasible_yaw is None:
            return None

        ori = get_gripper_pose_by_cup(feasible_yaw, roll_deg=GRASP_ROLL_DEG)
        if not self.plan_pose(bx, by, safe_z, ori):
            return None

        return feasible_yaw

    def _move_yaw_and_measure_error(self, bx, by, safe_z, yaw):
        feasible_yaw = self._move_to_yaw(bx, by, safe_z, yaw)
        if feasible_yaw is None:
            return None

        time.sleep(ALIGN_SETTLE_SEC)
        measurement = self._measure_mouth_up_error()
        if measurement is None:
            return None

        return feasible_yaw, measurement

    def _current_ee_yaw(self):
        T = get_ee_matrix(self.robot)
        return float(R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=False)[2])

    def _align_mouth_up_before_descent(self, bx, by, safe_z, grasp_yaw):
        """하강 전, 입구 벡터가 카메라 화면 위쪽(^)과 맞을 때까지 yaw만 보정."""
        log = self.get_logger()
        control_sign = None
        last_abs_error = None
        best_abs_error = None
        start_yaw = _normalize_angle(self._current_ee_yaw())
        best_yaw = start_yaw
        stable_count = 0
        align_yaw = start_yaw

        log.info(
            "[MouthAlign] 큰 첫 회전을 피하기 위해 현재 EE yaw에서 정렬을 시작합니다. "
            f"current_yaw={np.degrees(start_yaw):.1f}도, "
            f"vision_grasp_yaw={np.degrees(grasp_yaw):.1f}도, "
            f"delta={np.degrees(_normalize_angle(grasp_yaw - start_yaw)):+.1f}도"
        )

        for i in range(MOUTH_UP_MAX_ITERS):
            measurement = self._measure_mouth_up_error()
            if measurement is None:
                log.error(
                    "[MouthAlign] 빨간 점 방향을 못 찾았습니다. "
                    "입구 방향을 신뢰할 수 없어 하강하지 않습니다."
                )
                return None
            up_error = measurement["up_error"]
            grasp_left_error = self._grasp_left_error(align_yaw)

            log.info(
                f"[MouthAlign {i + 1}/{MOUTH_UP_MAX_ITERS}] "
                f"up_err={np.degrees(up_error):+.1f}도 "
                f"grasp_left_err={np.degrees(grasp_left_error):+.1f}도 "
                f"(허용 up ±{np.degrees(MOUTH_UP_TOL_RAD):.1f}도, "
                f"grasp ±{np.degrees(GRASP_LEFT_TOL_RAD):.1f}도, "
                f"stable {stable_count}/{MOUTH_UP_STABLE_COUNT}, "
                f"samples={measurement['sample_count']}, "
                f"spread={np.degrees(measurement['spread']):.1f}도, "
                f"source={measurement.get('source', 'marker')})"
            )

            if ENABLE_MOUTH_UP_LOCK and abs(up_error) <= MOUTH_UP_LOCK_TOL_RAD:
                log.info(
                    "[MouthAlign] 입구 벡터가 이미 12시 근처라 yaw를 잠급니다. "
                    f"up_err={np.degrees(up_error):+.1f}도 <= "
                    f"{np.degrees(MOUTH_UP_LOCK_TOL_RAD):.1f}도, "
                    "grasp_left_err 보정으로 12시 방향을 깨지 않고 현재 yaw로 진행합니다."
                )
                return align_yaw

            if abs(up_error) <= MOUTH_UP_TOL_RAD and abs(grasp_left_error) <= GRASP_LEFT_TOL_RAD:
                stable_count += 1
                if stable_count >= MOUTH_UP_STABLE_COUNT:
                    log.info("[MouthAlign] 노란색 12시 + 파란색 9시 조건이 안정적으로 들어왔습니다.")
                    return align_yaw
                last_abs_error = max(abs(up_error), abs(grasp_left_error))
                continue

            stable_count = 0
            current_abs_error = max(abs(up_error), abs(grasp_left_error))
            if best_abs_error is None or current_abs_error < best_abs_error:
                best_abs_error = current_abs_error
                best_yaw = align_yaw

            if control_sign is None:
                probe_step = MOUTH_UP_PROBE_STEP_RAD
                base_yaw = align_yaw
                base_up_error = up_error
                candidates = []
                last_probe_yaw = None

                for sign in (+1.0, -1.0):
                    result = self._move_yaw_and_measure_error(
                        bx, by, safe_z, _normalize_angle(base_yaw + sign * probe_step)
                    )
                    if result is None:
                        log.warn(
                            f"[MouthAlign] {sign:+.0f} 방향 probe 실패. "
                            "다른 방향을 확인합니다."
                        )
                        continue
                    candidate_yaw, candidate_measurement = result
                    last_probe_yaw = candidate_yaw
                    candidate_up_error = candidate_measurement["up_error"]
                    candidate_left_error = self._grasp_left_error(candidate_yaw)
                    candidate_score = max(abs(candidate_up_error), abs(candidate_left_error))
                    candidates.append(
                        (candidate_score, sign, candidate_yaw, candidate_up_error, candidate_left_error)
                    )

                    if ENABLE_MOUTH_UP_LOCK and abs(candidate_up_error) <= MOUTH_UP_LOCK_TOL_RAD:
                        log.info(
                            "[MouthAlign] probe 중 입구 벡터가 12시 lock 범위에 들어왔습니다. "
                            f"up_err={np.degrees(candidate_up_error):+.1f}도 <= "
                            f"{np.degrees(MOUTH_UP_LOCK_TOL_RAD):.1f}도, "
                            "해당 yaw로 진행합니다."
                        )
                        return candidate_yaw

                    if (
                        abs(candidate_up_error) <= MOUTH_UP_TOL_RAD
                        and abs(candidate_left_error) <= GRASP_LEFT_TOL_RAD
                    ):
                        log.info("[MouthAlign] probe 중 노란색 12시 + 파란색 9시 조건이 들어왔습니다.")
                        return candidate_yaw

                if not candidates:
                    log.error(
                        "[MouthAlign] +yaw/-yaw 양쪽 probe 모두 실패했습니다. "
                        "입구 방향을 신뢰할 수 없어 하강하지 않습니다."
                    )
                    return None

                candidates.sort(key=lambda item: item[0])
                _, best_sign, best_probe_yaw, best_probe_up_error, best_probe_left_error = candidates[0]
                control_sign = best_sign
                align_yaw = best_probe_yaw
                if (
                    last_probe_yaw is not None
                    and abs(_normalize_angle(last_probe_yaw - best_probe_yaw)) > np.radians(0.5)
                ):
                    moved_yaw = self._move_to_yaw(bx, by, safe_z, best_probe_yaw)
                    if moved_yaw is None:
                        log.error("[MouthAlign] 선택한 probe yaw로 복귀 실패. 하강하지 않습니다.")
                        return None
                    align_yaw = moved_yaw
                    time.sleep(ALIGN_SETTLE_SEC)
                up_error = best_probe_up_error
                grasp_left_error = best_probe_left_error
                current_abs_error = max(abs(up_error), abs(grasp_left_error))
                log.info(
                    "[MouthAlign] +yaw/-yaw 양방향 probe 결과 선택: "
                    f"best_up={np.degrees(best_probe_up_error):+.1f}도, "
                    f"best_grasp={np.degrees(best_probe_left_error):+.1f}도, "
                    f"control_sign={control_sign:+.0f}"
                )

                if abs(base_up_error) >= MOUTH_UP_OPPOSITE_THRESH_RAD:
                    jump_raw = control_sign * MOUTH_UP_OPPOSITE_GAIN * base_up_error
                    jump_step = float(
                        np.clip(
                            jump_raw,
                            -MOUTH_UP_OPPOSITE_MAX_STEP_RAD,
                            MOUTH_UP_OPPOSITE_MAX_STEP_RAD,
                        )
                    )
                    next_yaw = _normalize_angle(align_yaw + jump_step)
                    log.warn(
                        "[MouthAlign] 입구 벡터가 4~7시 방향처럼 거의 반대입니다. "
                        f"큰 보정 1회 적용: base_up={np.degrees(base_up_error):+.1f}도, "
                        f"jump={np.degrees(jump_step):+.1f}도"
                    )
                    moved_yaw = self._move_to_yaw(bx, by, safe_z, next_yaw)
                    if moved_yaw is None:
                        log.warn(
                            f"[MouthAlign] 큰 보정 yaw={np.degrees(next_yaw):.1f}도 이동 실패. "
                            "선택된 probe yaw에서 미세조정을 계속합니다."
                        )
                    else:
                        align_yaw = moved_yaw
                        last_abs_error = current_abs_error
                        time.sleep(ALIGN_SETTLE_SEC)
                        continue

            is_coarse = current_abs_error >= MOUTH_UP_COARSE_THRESH_RAD
            gain = MOUTH_UP_COARSE_GAIN if is_coarse else MOUTH_UP_GAIN
            max_step = MOUTH_UP_COARSE_MAX_STEP_RAD if is_coarse else MOUTH_UP_MAX_STEP_RAD
            if abs(grasp_left_error) > GRASP_LEFT_TOL_RAD and abs(up_error) <= MOUTH_UP_TOL_RAD:
                raw_step = -gain * grasp_left_error
            else:
                raw_step = control_sign * gain * up_error
            step = float(np.clip(raw_step, -max_step, max_step))
            if abs(step) < MOUTH_UP_MIN_STEP_RAD:
                step = float(np.sign(step if step != 0.0 else raw_step) * MOUTH_UP_MIN_STEP_RAD)
            if (
                last_abs_error is not None
                and current_abs_error > last_abs_error + MOUTH_UP_WORSEN_THRESH_RAD
            ):
                step *= 0.5
                log.warn("[MouthAlign] 직전보다 오차가 커져 이번 yaw step을 절반으로 줄입니다.")

            next_yaw = _normalize_angle(align_yaw + step)
            moved_yaw = self._move_to_yaw(bx, by, safe_z, next_yaw)
            if moved_yaw is None:
                log.warn(
                    f"[MouthAlign] 다음 yaw={np.degrees(next_yaw):.1f}도 이동 실패. "
                    "현재까지 가장 좋은 yaw로 진행합니다."
                )
                return best_yaw if best_abs_error is not None else align_yaw
            align_yaw = moved_yaw
            last_abs_error = current_abs_error
            time.sleep(ALIGN_SETTLE_SEC)
            log.info(
                f"[MouthAlign] {'coarse' if is_coarse else 'fine'} "
                f"yaw 보정량={np.degrees(step):+.1f}도 "
                f"-> 다음 yaw={np.degrees(align_yaw):.1f}도"
            )

        log.warn("[MouthAlign] 정렬이 완벽하지 않지만 가장 좋은 yaw로 잡기 동작을 진행합니다.")
        if best_abs_error is not None and best_abs_error <= MOUTH_UP_LOOSE_TOL_RAD:
            return best_yaw
        return align_yaw

    def _pick_and_straighten(self, bx, by, bz, cup_theta, grasp_yaw=None):
        log = self.get_logger()
        
        if grasp_yaw is None:
            grasp_yaw = cup_theta
        close_axis_theta = grasp_yaw + GRIPPER_CLOSE_AXIS_OFFSET_RAD
        target_ori = get_gripper_pose_by_cup(grasp_yaw, roll_deg=GRASP_ROLL_DEG)

     
        TABLE_Z = 0.0 
        floor_z = TABLE_Z


        Z_OFFSET = cfg.Z_OFFSET  # 0.20m (20cm)

        PICK_CLEARANCE = 0.02 
        
        pick_z = floor_z + CUP_RADIUS_M + Z_OFFSET + PICK_CLEARANCE
        place_z = max(floor_z + PLACE_RELEASE_CLEARANCE_M, cfg.SAFE_Z_MIN)
        pre_release_z = max(floor_z + PLACE_PRE_RELEASE_CLEARANCE_M, cfg.SAFE_Z_MIN)
        
        safe_z = float(np.clip(OBSERVE_SAFE_Z_M, cfg.SAFE_Z_MIN, cfg.SAFE_Z_MAX))
        
        log.info(
            f"== 컵 구출 시퀀스 준비 "
            f"(입구각: {np.degrees(cup_theta):.1f}도, "
            f"명령 yaw: {np.degrees(grasp_yaw):.1f}도, "
            f"닫힘축: {np.degrees(close_axis_theta):.1f}도, "
            f"roll: {GRASP_ROLL_DEG:.1f}도) =="
        )
          

        observe_bx, observe_by = self._observe_xy_from_cup(bx, by)
        log.info(
            f"[1-1] 컵에서 떨어진 관측 높이로 상공 진입 "
            f"(cup=({bx:.3f}, {by:.3f}) -> "
            f"observe=({observe_bx:.3f}, {observe_by:.3f}), Z={safe_z:.3f}m)"
        )

        T = get_ee_matrix(self.robot)
        qx, qy, qz, qw = R.from_matrix(T[:3, :3]).as_quat()
        current_yaw = float(R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=False)[2])
        current_ori = {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)}

        if not self.plan_pose(observe_bx, observe_by, safe_z, current_ori):
            log.error("[1-1] 상공 진입 실패. 시퀀스를 중단합니다.")
            return
        time.sleep(1.0)


        if ENABLE_PRE_GRASP_REALIGN:
            log.info("[1-2] 카메라 하향 자세를 유지하며 컵 뒤쪽 관측 위치로 XY 재정렬")
            aligned = self._align_above_cup(
                bx, by, cup_theta, grasp_yaw, safe_z, current_ori)
            if aligned is None:
                return
            bx, by, cup_theta, grasp_yaw = aligned
            target_ori = get_gripper_pose_by_cup(grasp_yaw, roll_deg=GRASP_ROLL_DEG)
        else:
            log.info("[Align] 카메라 하향 XY 재정렬은 비활성화되어 있습니다.")

        if ENABLE_MOUTH_UP_ALIGN:
            log.info("[1-3] 하강 전 입구 벡터를 카메라 화면 위쪽 기준으로 yaw 정렬")
            align_bx, align_by = self._observe_xy_from_cup(bx, by)
            aligned_yaw = self._align_mouth_up_before_descent(
                align_bx, align_by, safe_z, grasp_yaw)
            if aligned_yaw is None:
                log.error("[1-3] 입구 방향 정렬 실패. 하강하지 않습니다.")
                return
            grasp_yaw = aligned_yaw
            target_ori = get_gripper_pose_by_cup(grasp_yaw, roll_deg=GRASP_ROLL_DEG)
            close_axis_theta = grasp_yaw + GRIPPER_CLOSE_AXIS_OFFSET_RAD
            log.info(
                f"[1-3] 최종 yaw={np.degrees(grasp_yaw):.1f}도, "
                f"닫힘축={np.degrees(close_axis_theta):.1f}도"
            )
        else:
            log.info("[1-3] 입구 벡터 yaw 정렬 비활성화. 계산된 파지 yaw로 회전")
            if not self.plan_pose(bx, by, safe_z, target_ori):
                log.error("[1-3] 컵 파지 방향 회전 실패. 시퀀스를 중단합니다.")
                return
        time.sleep(0.5)

        log.info("[1-4] 파지 직전 중앙점 재추정")
        if self.color_image is not None:
            final_est = self._estimate_target_from_frame(self.color_image.copy())
            if final_est is not None:
                nbx, nby, _ = final_est["base"]
                log.info(
                    f"[1-4] 중앙점 갱신: ({bx:.3f}, {by:.3f}) -> ({nbx:.3f}, {nby:.3f})"
                )
                bx, by = nbx, nby
                if not self.plan_pose(bx, by, safe_z, target_ori):
                    log.error("[1-4] 재추정 위치로 수평 이동 실패. 시퀀스를 중단합니다.")
                    return
                time.sleep(0.3)
            else:
                log.warn("[1-4] 재추정 실패. 기존 좌표로 진행합니다.")
        else:
            log.warn("[1-4] 카메라 프레임 없음. 기존 좌표로 진행합니다.")

        log.info("[1-5] 하강 전 컵 가시성 최종 확인")
        visible_est = self._ensure_cup_visible(
            bx, by, safe_z, current_ori,
            max_retries=ENSURE_VISIBLE_MAX_RETRIES
        )
        if visible_est is None:
            log.error("[1-5] 컵을 다시 확인하지 못했습니다. 하강하지 않습니다.")
            return
        else:
            mouth = visible_est["mouth"]
            if mouth["found"]:
                final_up_error = _mouth_up_error(mouth["marker_theta"])
                sx, sy = mouth["sticker_center"]
                x1, y1, x2, y2 = visible_est["target"]["box"]
                margin = MARKER_EDGE_MARGIN_PX
                if (
                    sx <= x1 + margin or sx >= x2 - margin
                    or sy <= y1 + margin or sy >= y2 - margin
                ):
                    if final_up_error is not None and abs(final_up_error) <= MARKER_EDGE_ALLOW_UP_TOL_RAD:
                        log.warn(
                            "[1-5] 빨간 점이 bbox 가장자리에 있지만 "
                            f"입구 오차가 {np.degrees(final_up_error):+.1f}도로 작아 하강을 진행합니다."
                        )
                    else:
                        log.error("[1-5] 하강 직전 빨간 점이 bbox 가장자리에 걸려 있습니다. 하강하지 않습니다.")
                        return
            else:
                log.error("[1-5] 하강 직전 빨간 점이 보이지 않습니다. 하강하지 않습니다.")
                return
            nbx, nby, _ = visible_est["base"]
            bx, by = nbx, nby
            latest_yaw = visible_est["grasp_yaw"]
            yaw_err = abs(_normalize_angle(latest_yaw - grasp_yaw))
            if yaw_err > GRASP_YAW_VERIFY_TOL_RAD:
                log.warn(
                    f"[1-5] 최신 검출 yaw와 명령 yaw 차이가 큽니다 "
                    f"({np.degrees(yaw_err):.1f}도 > {np.degrees(GRASP_YAW_VERIFY_TOL_RAD):.1f}도). "
                    "이미 정렬된 yaw를 유지합니다."
                )
            target_ori = get_gripper_pose_by_cup(grasp_yaw, roll_deg=GRASP_ROLL_DEG)

            grasp_bx, grasp_by = self._body_grasp_xy_from_cup(bx, by, grasp_yaw)
            if abs(grasp_bx - bx) > 1e-6 or abs(grasp_by - by) > 1e-6:
                log.info(
                    "[1-5] 입구 쪽 파지를 피하기 위해 몸통 쪽으로 최종 파지점 보정: "
                    f"center=({bx:.3f}, {by:.3f}) -> "
                    f"grasp=({grasp_bx:.3f}, {grasp_by:.3f}), "
                    f"offset={GRASP_BODY_OFFSET_M:.3f}m"
                )
                bx, by = grasp_bx, grasp_by
        # 가시성 확인 중 관측 자세로 복귀했을 수 있으므로 파지 방향으로 재진입
        if not self.plan_pose(bx, by, safe_z, target_ori):
            log.error("[1-5] 파지 방향 자세로 복귀 실패. 시퀀스를 중단합니다.")
            return
        time.sleep(0.3)

        log.info("[2] 컵 집기 시작")
        if not self.plan_pose(bx, by, pick_z, target_ori):
            retry_pick_z = min(pick_z + 0.025, safe_z)
            log.warn(
                f"[2] 목표 픽 높이 Z={pick_z:.3f}m 이동 실패. "
                f"조금 높은 Z={retry_pick_z:.3f}m에서 한 번 더 잡기를 시도합니다."
            )
            if not self.plan_pose(bx, by, retry_pick_z, target_ori):
                log.error("[2] 픽 자세 이동 재시도 실패. 그리퍼를 닫지 않고 중단합니다.")
                return
        self.gripper.close_gripper()
        log.info("[2] 컵 집기 완료")
        log.info(f"[2-1] 컵을 잡은 상태로 {GRASP_HOLD_SEC:.1f}초 정지 후 리프트업합니다.")
        time.sleep(max(0.0, GRASP_HOLD_SEC))

        log.info("[3] Lift Up (다시 바닥 기준 25cm 상공으로 리프트업)")
        if not self.plan_pose(bx, by, safe_z, target_ori):
            log.error("[3] 리프트업 실패. 시퀀스를 중단합니다.")
            return
        time.sleep(1.0)

        log.info(
            "[4] 컵 세우기/직립화 궤적은 비활성화되어 있습니다. "
            "잡기와 리프트업까지만 수행하고 observe/home 복귀 단계로 넘어갑니다."
        )
        log.info("== 시퀀스 완료: grasp + lift only ==")
        return True

def main(args=None):
    run_node(YoloCupUprightingNode)


if __name__ == "__main__":
    main()
