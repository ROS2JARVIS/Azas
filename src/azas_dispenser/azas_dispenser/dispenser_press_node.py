#!/usr/bin/env python3
import time

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
        self.taught_posx_by_name = {
            "red": [
                float(v)
                for v in get_param(
                    self.node,
                    "red_top_posx",
                    [732.102, 64.331, 375.813, 174.047, -118.164, -149.737],
                )
            ],
            "green": [
                float(v)
                for v in get_param(
                    self.node,
                    "green_top_posx",
                    [733.471, 3.988, 379.210, 168.569, -117.133, -149.816],
                )
            ],
            "yellow": [
                float(v)
                for v in get_param(
                    self.node,
                    "yellow_top_posx",
                    [736.923, -54.696, 398.958, 164.238, -114.838, -150.599],
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
        self.press_depth_mm = float(get_param(self.node, "press_depth", 0.025)) * 1000.0
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
                self.logger.info(f"Waiting for {label} service")

    def call_service(self, client, request, label):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            self.logger.error(f"{label} failed: {future.exception()}")
            return False
        if not future.result().success:
            self.logger.error(f"{label} returned success=false")
            return False
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
                "Requested pose was outside workspace. "
                f"Clamped from ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm to "
                f"({safe_x:.1f}, {safe_y:.1f}, {safe_z:.1f}) mm."
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
        self.logger.info(f"{label}: waiting for motion completion")
        return self.call_service(self.move_wait, req, f"{label} wait")

    def read_current_posx(self):
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE

        future = self.get_current_posx.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        if result is None:
            self.logger.error(f"get_current_posx failed: {future.exception()}")
            return None
        if not result.success:
            self.logger.error("get_current_posx returned success=false")
            return None
        if not result.task_pos_info:
            self.logger.error("get_current_posx returned empty task_pos_info")
            return None

        data = list(result.task_pos_info[0].data)
        if len(data) < 6:
            self.logger.error(f"get_current_posx returned too few values: {data}")
            return None

        pose = data[:6]
        self.logger.info(
            "Current TCP pose from controller: "
            f"[{pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}, "
            f"{pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f}]"
        )
        return pose

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
                f"Using taught {self.target_dispenser} dispenser top pose. "
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
            ]
            return steps
        elif self.use_press_ready_pose:
            x_mm = self.dispenser_x_mm
            y_mm = self.dispenser_y_mm + self.dispenser_y_offset_mm
            top_z = self.dispenser_top_z_mm
            approach_z = top_z + self.approach_height_mm
            pressed_z = top_z - self.press_depth_mm

            self.logger.info(
                "Using saved press-ready joint pose. "
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
                "Using HOME/current TCP pose as dispenser approach pose. "
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
                "Using fixed dispenser position with HOME TCP orientation. "
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

        if self.use_press_ready_pose:
            if not self.movej(self.press_ready_joints_deg, "move to press-ready pose"):
                return False
            if not self.wait_for_motion_done("move to press-ready pose"):
                return False

        steps = self.build_press_steps()
        if steps is None:
            return False

        for x_mm, y_mm, z_mm, label in steps:
            if not self.movel(x_mm, y_mm, z_mm, label):
                return False
            if not self.wait_for_motion_done(label):
                return False
            if label == "approach above dispenser" and self.approach_pause_seconds > 0.0:
                self.logger.info(
                    f"Pausing at approach for {self.approach_pause_seconds:.2f} seconds"
                )
                time.sleep(self.approach_pause_seconds)
            if label == "press dispenser pump" and self.hold_seconds > 0.0:
                self.logger.info(f"Holding press for {self.hold_seconds:.2f} seconds")
                time.sleep(self.hold_seconds)

        if self.return_home:
            if not self.movej(self.home_joints_deg, "return to HOME"):
                return False
            return self.wait_for_motion_done("return to HOME")

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
