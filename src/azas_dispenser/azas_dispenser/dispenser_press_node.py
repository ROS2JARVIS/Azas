#!/usr/bin/env python3
import time

from azas_interfaces.srv import SetGripper
import rclpy
from dsr_msgs2.srv import GetCurrentPosx, MoveJoint, MoveLine, MoveWait


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
        self.gripper_client = self.node.create_client(
            SetGripper,
            self.gripper_service_name,
        )

    def destroy(self):
        self.node.destroy_node()

    def wait_for_services(self):
        for client, label in (
            (self.move_joint, "motion/move_joint"),
            (self.move_line, "motion/move_line"),
            (self.move_wait, "motion/move_wait"),
            (self.get_current_posx, "aux_control/get_current_posx"),
        ):
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                self.logger.info(f"서비스 {label} 를 기다리는 중")

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

    def movel(self, x_mm, y_mm, z_mm, label):
        safe_x = clamp(x_mm, SAFE_X_MIN_MM, SAFE_X_MAX_MM)
        safe_y = clamp(y_mm, SAFE_Y_MIN_MM, SAFE_Y_MAX_MM)
        safe_z = clamp(z_mm, SAFE_Z_MIN_MM, SAFE_Z_MAX_MM)

        if (safe_x, safe_y, safe_z) != (x_mm, y_mm, z_mm):
            self.logger.warning(
                "요청한 포즈가 작업 공간을 벗어났습니다. "
                f"({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm 에서 "
                f"({safe_x:.1f}, {safe_y:.1f}, {safe_z:.1f}) mm 로 클램프했습니다."
            )

        req = MoveLine.Request()
        req.pos = [safe_x, safe_y, safe_z, self.rx, self.ry, self.rz]
        req.vel = [self.line_velocity, self.line_velocity]
        req.acc = [self.line_acceleration, self.line_acceleration]
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
            (x_mm, y_mm, approach_z, "approach above dispenser"),
            (x_mm, y_mm, top_z, "move to dispenser top"),
            (x_mm, y_mm, pressed_z, "press dispenser pump"),
            (x_mm, y_mm, approach_z, "retreat above dispenser"),
        ]

        if home_lift_step is not None:
            return [home_lift_step] + steps

        return steps

    def run(self):
        if self.press_depth_mm <= 0.0:
            self.logger.error("press_depth must be greater than 0.0 m.")
            return False

        self.wait_for_services()

        if self.move_home_first:
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

        for idx, (x_mm, y_mm, z_mm, label) in enumerate(steps, start=1):
            self.logger.info(f"단계 {idx}/{len(steps)}: 시작 '{label}' -> x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm")
            if not self.movel(x_mm, y_mm, z_mm, label):
                self.logger.error(f"{label} 단계에서 movel 실패")
                return False
            if not self.wait_for_motion_done(label):
                self.logger.error(f"{label} 단계의 모션이 완료되지 않았습니다")
                return False
            self.verify_reached_pose([x_mm, y_mm, z_mm, self.rx, self.ry, self.rz], label)
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
    try:
        node.run()
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
