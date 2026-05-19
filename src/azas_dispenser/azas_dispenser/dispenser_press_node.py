#!/usr/bin/env python3
import time

from azas_interfaces.srv import SetGripper
import rclpy
from dsr_msgs2.srv import (
    GetCurrentPosx,
    GetCurrentTcp,
    MoveJoint,
    MoveLine,
    MoveWait,
    SetCurrentTcp,
)


DR_BASE = 0
MOVE_MODE_ABSOLUTE = 0
SYNC = 0
BLENDING_SPEED_TYPE_DUPLICATE = 0

SAFE_X_MIN_MM = 0.0
SAFE_X_MAX_MM = 800.0
SAFE_Y_MIN_MM = -300.0
SAFE_Y_MAX_MM = 300.0
SAFE_Z_MIN_MM = 270.0
SAFE_Z_MAX_MM = 750.0


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def service_name(prefix, name):
    clean_prefix = prefix.strip("/")
    clean_name = name.strip("/")
    if not clean_prefix:
        return f"/{clean_name}"
    return f"/{clean_prefix}/{clean_name}"


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


class DispenserPressNode:
    def __init__(self):
        self.node = rclpy.create_node("dispenser_press_node")
        self.logger = self.node.get_logger()

        self.service_prefix = str(get_param(self.node, "service_prefix", ""))
        self.tcp_name = str(get_param(self.node, "tcp_name", "")).strip()
        self.restore_tcp_after_run = bool(get_param(self.node, "restore_tcp_after_run", True))
        self.previous_tcp_name = None
        self.require_tcp_for_taught_posx = bool(
            get_param(self.node, "require_tcp_for_taught_posx", True)
        )
        self.move_home_first = bool(get_param(self.node, "move_home_first", True))
        self.return_home = bool(get_param(self.node, "return_home", True))
        self.use_home_as_reference = bool(
            get_param(self.node, "use_home_as_reference", True)
        )
        self.keep_home_orientation = bool(
            get_param(self.node, "keep_home_orientation", True)
        )
        self.use_press_ready_pose = bool(
            get_param(self.node, "use_press_ready_pose", False)
        )
        self.use_taught_posx = bool(get_param(self.node, "use_taught_posx", False))
        self.target_dispenser = str(get_param(self.node, "target_dispenser", "red"))
        self.close_gripper_at_home = bool(
            get_param(self.node, "close_gripper_at_home", True)
        )
        self.gripper_service_name = str(
            get_param(self.node, "gripper_service", "/azas/gripper/open_close")
        )
        self.gripper_close_width_m = float(
            get_param(self.node, "gripper_close_width", 0.0)
        )
        self.gripper_close_force_n = float(
            get_param(self.node, "gripper_close_force", 20.0)
        )
        self.gripper_wait_timeout_sec = float(
            get_param(self.node, "gripper_wait_timeout", 2.0)
        )
        self.service_wait_timeout_sec = float(
            get_param(self.node, "service_wait_timeout_sec", 10.0)
        )
        self.pose_position_tolerance_mm = float(
            get_param(self.node, "pose_position_tolerance_mm", 5.0)
        )
        self.pose_orientation_tolerance_deg = float(
            get_param(self.node, "pose_orientation_tolerance_deg", 5.0)
        )
        self.strict_pose_verification = bool(
            get_param(self.node, "strict_pose_verification", False)
        )
        self.taught_posx_by_name = {
            "red": [
                float(v)
                for v in get_param(
                    self.node,
                    "red_top_posx",
                    [732.102, 64.331, 379.151, 174.047, -118.164, -149.737],
                )
            ],
            "green": [
                float(v)
                for v in get_param(
                    self.node,
                    "green_top_posx",
                    [733.471, 3.988, 379.151, 168.569, -117.133, -149.816],
                )
            ],
            "yellow": [
                float(v)
                for v in get_param(
                    self.node,
                    "yellow_top_posx",
                    [736.923, -54.696, 379.151, 164.238, -114.838, -150.599],
                )
            ],
            "blue": [
                float(v)
                for v in get_param(
                    self.node,
                    "blue_top_posx",
                    [730.658, -109.868, 379.151, 158.766, -114.912, -156.963],
                )
            ],
        }
        self.dispenser_x_mm = float(get_param(self.node, "dispenser_x", 0.50)) * 1000.0
        self.dispenser_y_mm = float(get_param(self.node, "dispenser_y", 0.00)) * 1000.0
        self.dispenser_y_offset_mm = (
            float(get_param(self.node, "dispenser_y_offset", 0.05)) * 1000.0
        )
        self.dispenser_top_z_mm = (
            float(get_param(self.node, "dispenser_top_z", 0.37)) * 1000.0
        )
        self.approach_height_mm = (
            float(get_param(self.node, "approach_height", 0.10)) * 1000.0
        )
        self.transit_height_mm = (
            float(get_param(self.node, "transit_height", 0.10)) * 1000.0
        )
        self.home_lift_height_mm = (
            float(get_param(self.node, "home_lift_height", 0.05)) * 1000.0
        )
        self.press_depth_mm = float(get_param(self.node, "press_depth", 0.04)) * 1000.0
        self.hold_seconds = float(get_param(self.node, "hold_seconds", 0.5))
        self.approach_pause_seconds = float(
            get_param(self.node, "approach_pause_seconds", 0.5)
        )

        self.rx = float(get_param(self.node, "rx", 180.0))
        self.ry = float(get_param(self.node, "ry", 0.0))
        self.rz = float(get_param(self.node, "rz", 180.0))

        self.home_joints_deg = [
            float(v)
            for v in get_param(
                self.node,
                "home_joints_deg",
                [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
            )
        ]
        self.press_ready_joints_deg = [
            float(v)
            for v in get_param(
                self.node,
                "press_ready_joints_deg",
                [6.58, 6.94, 57.71, -15.02, 26.12, -76.44],
            )
        ]

        self.joint_velocity = float(get_param(self.node, "joint_velocity", 20.0))
        self.joint_acceleration = float(get_param(self.node, "joint_acceleration", 20.0))
        self.line_velocity = float(get_param(self.node, "line_velocity", 30.0))
        self.line_acceleration = float(get_param(self.node, "line_acceleration", 50.0))
        self.travel_line_velocity = float(
            get_param(self.node, "travel_line_velocity", self.line_velocity)
        )
        self.travel_line_acceleration = float(
            get_param(self.node, "travel_line_acceleration", self.line_acceleration)
        )
        self.pre_home_retreat_before_home = bool(
            get_param(self.node, "pre_home_retreat_before_home", False)
        )
        # Press starts are often requested while the gripper is still near the cup/
        # dispenser handoff area.  A direct HOME movej from there can sweep the
        # gripper through the cup.  Default direction is -X in base coordinates:
        # dispenser taught poses are near x~=730 mm and HOME is near x~=370 mm, so
        # this pulls back toward the robot before the joint HOME transition.
        self.pre_home_retreat_dx_mm = float(
            get_param(self.node, "pre_home_retreat_dx_mm", -120.0)
        )
        self.pre_home_retreat_dy_mm = float(
            get_param(self.node, "pre_home_retreat_dy_mm", 0.0)
        )
        self.pre_home_retreat_min_z_mm = float(
            get_param(self.node, "pre_home_retreat_min_z_mm", 0.0)
        )
        self.pre_home_retreat_min_current_x_mm = float(
            get_param(self.node, "pre_home_retreat_min_current_x_mm", 450.0)
        )
        self.pre_home_retreat_velocity = float(
            get_param(self.node, "pre_home_retreat_velocity", min(self.travel_line_velocity, 25.0))
        )
        self.pre_home_retreat_acceleration = float(
            get_param(
                self.node,
                "pre_home_retreat_acceleration",
                min(self.travel_line_acceleration, 30.0),
            )
        )

        self.move_joint = self.node.create_client(
            MoveJoint,
            service_name(self.service_prefix, "motion/move_joint"),
        )
        self.move_line = self.node.create_client(
            MoveLine,
            service_name(self.service_prefix, "motion/move_line"),
        )
        self.move_wait = self.node.create_client(
            MoveWait,
            service_name(self.service_prefix, "motion/move_wait"),
        )
        self.get_current_posx = self.node.create_client(
            GetCurrentPosx,
            service_name(self.service_prefix, "aux_control/get_current_posx"),
        )
        self.set_current_tcp = self.node.create_client(
            SetCurrentTcp,
            service_name(self.service_prefix, "tcp/set_current_tcp"),
        )
        self.get_current_tcp = self.node.create_client(
            GetCurrentTcp,
            service_name(self.service_prefix, "tcp/get_current_tcp"),
        )
        self.gripper_client = self.node.create_client(
            SetGripper,
            self.gripper_service_name,
        )

    def destroy(self):
        self.node.destroy_node()

    def wait_for_services(self):
        deadline = time.monotonic() + max(self.service_wait_timeout_sec, 0.1)
        required_clients = [
            (self.move_joint, "motion/move_joint"),
            (self.move_line, "motion/move_line"),
            (self.move_wait, "motion/move_wait"),
            (self.get_current_posx, "aux_control/get_current_posx"),
        ]
        if self.tcp_name or (self.use_taught_posx and self.require_tcp_for_taught_posx):
            required_clients.extend(
                [
                    (self.set_current_tcp, "tcp/set_current_tcp"),
                    (self.get_current_tcp, "tcp/get_current_tcp"),
                ]
            )

        for client, label in required_clients:
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                if time.monotonic() > deadline:
                    self.logger.error(
                        f"서비스 {label} 를 {self.service_wait_timeout_sec:.1f}초 안에 찾지 못했습니다"
                    )
                    return False
                self.logger.info(f"서비스 {label} 를 기다리는 중")
        return True

    def call_service(self, client, request, label):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            self.logger.error(f"{label} 호출 실패: {future.exception()}")
            return False
        if not future.result().success:
            self.logger.error(f"{label} 가 success=false를 반환했습니다")
            return False
        return True

    def current_tcp_name(self):
        get_req = GetCurrentTcp.Request()
        future = self.get_current_tcp.call_async(get_req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is None:
            self.logger.error(f"tcp/get_current_tcp 호출 실패: {future.exception()}")
            return None
        if not result.success:
            self.logger.error("tcp/get_current_tcp 가 success=false를 반환했습니다")
            return None
        return str(result.info).strip()

    def set_tcp_name(self, name, label):
        req = SetCurrentTcp.Request()
        req.name = str(name).strip()
        self.logger.info(f"{label}: Doosan current TCP 설정: {req.name if req.name else '<empty/default>'}")
        return self.call_service(self.set_current_tcp, req, label)

    def configure_tcp(self):
        if not self.tcp_name:
            if self.use_taught_posx and self.require_tcp_for_taught_posx:
                self.logger.error(
                    "use_taught_posx=true 프레스는 Doosan controller TCP 이름이 필요합니다. "
                    "tcp_name 파라미터가 비어 있어 link_6/flange 기준 프레스를 막습니다."
                )
                return False
            self.logger.warning(
                "tcp_name이 비어 있습니다. 현재 컨트롤러 TCP를 그대로 사용합니다."
            )
            return True

        self.previous_tcp_name = self.current_tcp_name()
        if self.previous_tcp_name is None:
            return False
        self.logger.info(
            "프레스 전 Doosan TCP: "
            f"{self.previous_tcp_name if self.previous_tcp_name else '<empty/default>'}"
        )

        if self.previous_tcp_name == self.tcp_name:
            self.logger.info(
                f"요청 TCP '{self.tcp_name}' 가 이미 활성화되어 있어 "
                "tcp/set_current_tcp 호출을 건너뜁니다."
            )
            return True

        if not self.set_tcp_name(self.tcp_name, "tcp/set_current_tcp"):
            self.logger.error(
                f"Doosan controller가 TCP '{self.tcp_name}' 설정을 거부했습니다. "
                "티치펜던트/컨트롤러에 해당 TCP가 등록되어 있는지 확인하세요."
            )
            return False

        current_name = self.current_tcp_name()
        if current_name is None:
            return False
        self.logger.info(f"현재 Doosan TCP 확인: {current_name}")
        if current_name != self.tcp_name:
            self.logger.error(
                f"요청 TCP '{self.tcp_name}' 와 현재 TCP '{current_name}' 가 다릅니다. "
                "link_6/flange 기준 프레스를 막습니다."
            )
            return False
        return True

    def restore_tcp_if_needed(self):
        if not self.restore_tcp_after_run:
            return
        if self.previous_tcp_name is None or self.previous_tcp_name == self.tcp_name:
            return
        previous = self.previous_tcp_name
        self.logger.info(
            "프레스 후 Doosan TCP를 이전 값으로 복원합니다: "
            f"{previous if previous else '<empty/default>'}"
        )
        if not self.set_tcp_name(previous, "tcp/restore_previous_tcp"):
            self.logger.error(
                "프레스 후 TCP 복원 실패. 다음 link_6 기준 이동 전에 티치펜던트/컨트롤러 TCP를 확인하세요."
            )
            return
        current_name = self.current_tcp_name()
        if current_name != previous:
            self.logger.error(
                f"TCP 복원 확인 실패: current='{current_name}', expected='{previous}'"
            )
            return
        self.logger.info("Doosan TCP 복원 확인 완료")

    def close_gripper(self):
        if not self.close_gripper_at_home:
            return True

        self.logger.info(f"그리퍼 close 서비스 확인 중: {self.gripper_service_name}")
        if not self.gripper_client.wait_for_service(
            timeout_sec=self.gripper_wait_timeout_sec
        ):
            self.logger.warning(
                f"그리퍼 서비스 {self.gripper_service_name} 를 찾지 못했습니다. "
                "그리퍼 닫기 없이 디스펜서 동작을 계속합니다."
            )
            return True

        req = SetGripper.Request()
        req.command = "close"
        req.width_m = self.gripper_close_width_m
        req.force_n = self.gripper_close_force_n

        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is None:
            self.logger.error(f"그리퍼 close 호출 실패: {future.exception()}")
            return False
        if not result.success:
            self.logger.error(f"그리퍼 close 실패: {result.message}")
            return False

        self.logger.info(f"그리퍼 close 완료: {result.message}")
        return True

    def movej(self, joints_deg, label):
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = self.joint_velocity
        req.acc = self.joint_acceleration
        req.time = 0.0
        req.radius = 0.0
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        self.logger.info(f"{label}: movej {req.pos}")
        return self.call_service(self.move_joint, req, label)

    def movel(self, x_mm, y_mm, z_mm, label, rpy_deg=None, velocity=None, acceleration=None):
        safe_x = clamp(x_mm, SAFE_X_MIN_MM, SAFE_X_MAX_MM)
        safe_y = clamp(y_mm, SAFE_Y_MIN_MM, SAFE_Y_MAX_MM)
        safe_z = clamp(z_mm, SAFE_Z_MIN_MM, SAFE_Z_MAX_MM)
        rx, ry, rz = rpy_deg if rpy_deg is not None else (self.rx, self.ry, self.rz)

        if (safe_x, safe_y, safe_z) != (x_mm, y_mm, z_mm):
            self.logger.warning(
                "요청한 포즈가 작업 공간을 벗어났습니다. "
                f"({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm 에서 "
                f"({safe_x:.1f}, {safe_y:.1f}, {safe_z:.1f}) mm 로 클램프했습니다."
            )

        req = MoveLine.Request()
        req.pos = [safe_x, safe_y, safe_z, float(rx), float(ry), float(rz)]
        line_velocity = self.line_velocity if velocity is None else float(velocity)
        line_acceleration = self.line_acceleration if acceleration is None else float(acceleration)
        req.vel = [line_velocity, line_velocity]
        req.acc = [line_acceleration, line_acceleration]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        self.logger.info(f"{label}: movel {req.pos}")
        return self.call_service(self.move_line, req, label)

    def wait_for_motion_done(self, label):
        req = MoveWait.Request()
        self.logger.info(f"{label}: 모션 완료 대기 중")
        return self.call_service(self.move_wait, req, f"{label} wait")

    def movel_checked_no_clamp(
        self,
        x_mm,
        y_mm,
        z_mm,
        label,
        rpy_deg,
        velocity,
        acceleration,
        z_min_mm=50.0,
    ):
        if not (SAFE_X_MIN_MM <= x_mm <= SAFE_X_MAX_MM):
            self.logger.error(
                f"{label}: x={x_mm:.1f} mm가 허용 범위 "
                f"[{SAFE_X_MIN_MM:.1f}, {SAFE_X_MAX_MM:.1f}] mm 밖입니다."
            )
            return False
        if not (SAFE_Y_MIN_MM <= y_mm <= SAFE_Y_MAX_MM):
            self.logger.error(
                f"{label}: y={y_mm:.1f} mm가 허용 범위 "
                f"[{SAFE_Y_MIN_MM:.1f}, {SAFE_Y_MAX_MM:.1f}] mm 밖입니다."
            )
            return False
        if not (z_min_mm <= z_mm <= SAFE_Z_MAX_MM):
            self.logger.error(
                f"{label}: z={z_mm:.1f} mm가 허용 범위 "
                f"[{z_min_mm:.1f}, {SAFE_Z_MAX_MM:.1f}] mm 밖입니다."
            )
            return False

        rx, ry, rz = rpy_deg
        req = MoveLine.Request()
        req.pos = [float(x_mm), float(y_mm), float(z_mm), float(rx), float(ry), float(rz)]
        req.vel = [float(velocity), float(velocity)]
        req.acc = [float(acceleration), float(acceleration)]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = MOVE_MODE_ABSOLUTE
        req.blend_type = BLENDING_SPEED_TYPE_DUPLICATE
        req.sync_type = SYNC

        self.logger.info(f"{label}: checked movel(no clamp) {req.pos}")
        return self.call_service(self.move_line, req, label)

    def read_current_posx(self):
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE

        future = self.get_current_posx.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        if result is None:
            self.logger.error(f"get_current_posx 호출 실패: {future.exception()}")
            return None
        if not result.success:
            self.logger.error("get_current_posx가 success=false를 반환했습니다")
            return None
        if not result.task_pos_info:
            self.logger.error("get_current_posx가 빈 task_pos_info를 반환했습니다")
            return None

        data = list(result.task_pos_info[0].data)
        if len(data) < 6:
            self.logger.error(f"get_current_posx가 너무 적은 값을 반환했습니다: {data}")
            return None

        pose = data[:6]
        self.logger.info(
            "현재 Doosan TCP 포즈: "
            f"[{pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}, "
            f"{pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f}]"
        )
        return pose

    def shortest_angle_delta_deg(self, target_deg, actual_deg):
        return (actual_deg - target_deg + 180.0) % 360.0 - 180.0

    def verify_reached_pose(self, target_pose, label):
        actual_pose = self.read_current_posx()
        if actual_pose is None:
            return False

        dx = actual_pose[0] - target_pose[0]
        dy = actual_pose[1] - target_pose[1]
        dz = actual_pose[2] - target_pose[2]
        position_error = (dx * dx + dy * dy + dz * dz) ** 0.5
        drx = self.shortest_angle_delta_deg(target_pose[3], actual_pose[3])
        dry = self.shortest_angle_delta_deg(target_pose[4], actual_pose[4])
        drz = self.shortest_angle_delta_deg(target_pose[5], actual_pose[5])
        max_rpy_error = max(abs(drx), abs(dry), abs(drz))

        self.logger.info(
            f"{label}: 목표 도달 오차 "
            f"position={position_error:.2f} mm "
            f"(dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}), "
            f"rpy_max={max_rpy_error:.2f} deg "
            f"(drx={drx:.2f}, dry={dry:.2f}, drz={drz:.2f})"
        )
        if not self.strict_pose_verification:
            return True
        if position_error > self.pose_position_tolerance_mm:
            self.logger.error(
                f"{label}: 위치 오차 {position_error:.2f} mm가 허용값 "
                f"{self.pose_position_tolerance_mm:.2f} mm를 초과했습니다."
            )
            return False
        if max_rpy_error > self.pose_orientation_tolerance_deg:
            self.logger.error(
                f"{label}: 자세 오차 {max_rpy_error:.2f} deg가 허용값 "
                f"{self.pose_orientation_tolerance_deg:.2f} deg를 초과했습니다."
            )
            return False
        return True

    def retreat_before_home_if_needed(self):
        if not self.pre_home_retreat_before_home:
            return True

        current_pose = self.read_current_posx()
        if current_pose is None:
            return False

        current_x = current_pose[0]
        if current_x < self.pre_home_retreat_min_current_x_mm:
            self.logger.info(
                "pre-HOME retreat 생략: 현재 TCP x="
                f"{current_x:.1f} mm 가 기준 "
                f"{self.pre_home_retreat_min_current_x_mm:.1f} mm 보다 작아 "
                "이미 HOME/로봇 쪽 안전 영역에 있다고 판단했습니다."
            )
            return True

        target_x = current_pose[0] + self.pre_home_retreat_dx_mm
        target_y = current_pose[1] + self.pre_home_retreat_dy_mm
        target_z = max(current_pose[2], self.pre_home_retreat_min_z_mm)
        target_rpy = current_pose[3:6]
        label = "pre-HOME retreat away from cup"

        self.logger.info(
            "HOME movej 전에 컵/디스펜서 충돌 회피용 후퇴를 수행합니다: "
            f"dx={self.pre_home_retreat_dx_mm:.1f} mm, "
            f"dy={self.pre_home_retreat_dy_mm:.1f} mm, "
            f"target_z={target_z:.1f} mm, rpy 유지="
            f"({target_rpy[0]:.1f}, {target_rpy[1]:.1f}, {target_rpy[2]:.1f})"
        )
        if not self.movel_checked_no_clamp(
            target_x,
            target_y,
            target_z,
            label,
            target_rpy,
            self.pre_home_retreat_velocity,
            self.pre_home_retreat_acceleration,
        ):
            self.logger.error("pre-HOME retreat movel 실패")
            return False
        if not self.wait_for_motion_done(label):
            self.logger.error("pre-HOME retreat 모션이 완료되지 않았습니다")
            return False

        clamped_target = [
            target_x,
            target_y,
            target_z,
            target_rpy[0],
            target_rpy[1],
            target_rpy[2],
        ]
        if not self.verify_reached_pose(clamped_target, label):
            self.logger.error("pre-HOME retreat 목표 포즈 확인 실패")
            return False
        return True

    def build_press_steps(self):
        home_lift_step = None

        if self.use_taught_posx:
            current_pose = self.read_current_posx()
            if current_pose is None:
                return None

            top_pose = self.taught_posx_by_name.get(self.target_dispenser)
            if top_pose is None:
                self.logger.error(
                    f"Unknown target_dispenser '{self.target_dispenser}'. "
                    f"Choose one of {sorted(self.taught_posx_by_name)}."
                )
                return None
            if len(top_pose) < 6:
                self.logger.error(
                    f"{self.target_dispenser}_top_posx must have 6 values."
                )
                return None

            x_mm, y_mm, top_z, self.rx, self.ry, self.rz = top_pose[:6]
            approach_z = top_z + self.approach_height_mm
            pressed_z = top_z - self.press_depth_mm
            transit_z = max(current_pose[2], approach_z) + self.transit_height_mm

            self.logger.info(
                f"학습된 {self.target_dispenser} 디스펜서 top 포즈를 사용합니다. "
                f"top=({x_mm:.1f}, {y_mm:.1f}, {top_z:.1f}), "
                f"transit_z={transit_z:.1f} mm, approach_z={approach_z:.1f} mm, "
                f"pressed_z={pressed_z:.1f} mm, "
                f"rpy=({self.rx:.1f}, {self.ry:.1f}, {self.rz:.1f})"
            )

            steps = [
                (
                    current_pose[0],
                    current_pose[1],
                    transit_z,
                    "lift to transit height",
                ),
                (x_mm, y_mm, transit_z, "align above dispenser"),
                (x_mm, y_mm, approach_z, "descend to approach"),
                (x_mm, y_mm, top_z, "move to dispenser top"),
                (x_mm, y_mm, pressed_z, "press dispenser pump"),
                (x_mm, y_mm, approach_z, "retreat above dispenser"),
                (x_mm, y_mm, transit_z, "lift to return transit height"),
            ]
            return steps
        elif self.use_press_ready_pose:
            x_mm = self.dispenser_x_mm
            y_mm = self.dispenser_y_mm + self.dispenser_y_offset_mm
            top_z = self.dispenser_top_z_mm
            approach_z = top_z + self.approach_height_mm
            pressed_z = top_z - self.press_depth_mm

            self.logger.info(
                "저장된 press-ready 관절 포즈를 사용합니다. "
                f"target=({x_mm:.1f}, {y_mm:.1f}), "
                f"rpy=({self.rx:.1f}, {self.ry:.1f}, {self.rz:.1f})"
            )
        elif self.use_home_as_reference:
            current_pose = self.read_current_posx()
            if current_pose is None:
                return None

            x_mm, y_mm, approach_z, rx, ry, rz = current_pose
            self.rx = rx
            self.ry = ry
            self.rz = rz
            top_z = approach_z - self.approach_height_mm
            pressed_z = top_z - self.press_depth_mm

            self.logger.info(
                "HOME/현재 TCP 포즈를 디스펜서 접근 포즈로 사용합니다. "
                f"top_z={top_z:.1f} mm, pressed_z={pressed_z:.1f} mm"
            )
        else:
            if self.keep_home_orientation:
                current_pose = self.read_current_posx()
                if current_pose is None:
                    return None
                home_x, home_y, home_z = current_pose[:3]
                self.rx = current_pose[3]
                self.ry = current_pose[4]
                self.rz = current_pose[5]
                home_lift_step = (
                    home_x,
                    home_y,
                    home_z + self.home_lift_height_mm,
                    "lift above HOME",
                )

            x_mm = self.dispenser_x_mm
            y_mm = self.dispenser_y_mm + self.dispenser_y_offset_mm
            top_z = self.dispenser_top_z_mm
            approach_z = top_z + self.approach_height_mm
            pressed_z = top_z - self.press_depth_mm

            self.logger.info(
                "고정된 디스펜서 위치를 HOME TCP 방향과 함께 사용합니다. "
                f"target=({x_mm:.1f}, {y_mm:.1f}), "
                f"rpy=({self.rx:.1f}, {self.ry:.1f}, {self.rz:.1f})"
            )

        steps = [
            (x_mm, y_mm, approach_z, self.rx, self.ry, self.rz, "approach above dispenser"),
            (x_mm, y_mm, top_z, self.rx, self.ry, self.rz, "move to dispenser top"),
            (x_mm, y_mm, pressed_z, self.rx, self.ry, self.rz, "press dispenser pump"),
            (x_mm, y_mm, approach_z, self.rx, self.ry, self.rz, "retreat above dispenser"),
        ]

        if home_lift_step is not None:
            home_x, home_y, home_z, home_label = home_lift_step
            return [
                (home_x, home_y, home_z, self.rx, self.ry, self.rz, home_label)
            ] + steps

        return steps

    def run(self):
        if self.press_depth_mm <= 0.0:
            self.logger.error("press_depth must be greater than 0.0 m.")
            return False

        if not self.wait_for_services():
            return False
        if not self.configure_tcp():
            return False

        if self.move_home_first:
            if not self.retreat_before_home_if_needed():
                return False
            if not self.movej(self.home_joints_deg, "move to HOME"):
                return False
            if not self.wait_for_motion_done("move to HOME"):
                return False
            if not self.close_gripper():
                return False

        if self.use_press_ready_pose:
            if not self.movej(self.press_ready_joints_deg, "move to press-ready pose"):
                return False
            if not self.wait_for_motion_done("move to press-ready pose"):
                return False

        steps = self.build_press_steps()
        if steps is None:
            return False

        for idx, step in enumerate(steps, start=1):
            if len(step) == 4:
                x_mm, y_mm, z_mm, label = step
                rx, ry, rz = self.rx, self.ry, self.rz
            else:
                x_mm, y_mm, z_mm, rx, ry, rz, label = step
            is_press_down = label == "press dispenser pump"
            line_velocity = self.line_velocity if is_press_down else self.travel_line_velocity
            line_acceleration = (
                self.line_acceleration if is_press_down else self.travel_line_acceleration
            )
            self.logger.info(f"단계 {idx}/{len(steps)}: 시작 '{label}' -> x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm")
            if not self.movel(
                x_mm,
                y_mm,
                z_mm,
                label,
                (rx, ry, rz),
                velocity=line_velocity,
                acceleration=line_acceleration,
            ):
                self.logger.error(f"{label} 단계에서 movel 실패")
                return False
            if not self.wait_for_motion_done(label):
                self.logger.error(f"{label} 단계의 모션이 완료되지 않았습니다")
                return False
            if not self.verify_reached_pose([x_mm, y_mm, z_mm, rx, ry, rz], label):
                self.logger.error(f"{label} 단계가 목표 포즈에 도달하지 못했습니다")
                return False
            self.logger.info(f"단계 {idx}/{len(steps)}: 완료 '{label}'")
            if label == "approach above dispenser" and self.approach_pause_seconds > 0.0:
                self.logger.info(
                    f"접근 위치에서 {self.approach_pause_seconds:.2f}초간 대기합니다"
                )
                time.sleep(self.approach_pause_seconds)
            if label == "press dispenser pump" and self.hold_seconds > 0.0:
                self.logger.info(f"누르는 동작을 {self.hold_seconds:.2f}초간 유지합니다")
                time.sleep(self.hold_seconds)

        if self.return_home:
            self.logger.info(
                "복귀용 transit 높이 도달 후 HOME으로 복귀합니다."
            )
            if not self.movej(self.home_joints_deg, "return to HOME"):
                return False
            if not self.wait_for_motion_done("return to HOME"):
                return False
            self.logger.info("HOME 복귀 완료")
            return True

        return True


def main(args=None):
    rclpy.init(args=args)
    node = DispenserPressNode()
    ok = False
    try:
        ok = node.run()
    finally:
        try:
            node.restore_tcp_if_needed()
        finally:
            node.destroy()
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
