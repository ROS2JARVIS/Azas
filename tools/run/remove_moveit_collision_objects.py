#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

import rclpy
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

DEFAULT_IDS = [
    "dispenser_body_box",
    "dispenser_1_body_box_v2",
    "dispenser_2_body_box_v2",
    "dispenser_3_body_box_v2",
    "dispenser_4_body_box_v2",
    "dispenser_head_box",
    "dispenser_head_nozzle_merged_vertical_box",
    "dispenser_head_nozzle_merged_horizontal_spout_box",
    "dispenser_1_head_nozzle_box",
    "dispenser_2_head_nozzle_box",
    "dispenser_3_head_nozzle_box",
    "dispenser_4_head_nozzle_box",
    "side_grip_workspace_x_min_wall",
    "side_grip_workspace_x_max_wall",
    "side_grip_workspace_y_min_wall",
    "side_grip_workspace_y_max_wall",
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Remove stale collision objects from MoveIt planning scene.")
    parser.add_argument("--service", default="/apply_planning_scene")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ids", default=",".join(DEFAULT_IDS))
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args = parser.parse_args()

    object_ids = [item.strip() for item in args.ids.split(",") if item.strip()]
    rclpy.init()
    node = rclpy.create_node("azas_remove_moveit_collision_objects")
    try:
        client = node.create_client(ApplyPlanningScene, args.service)
        if not client.wait_for_service(timeout_sec=args.timeout_sec):
            node.get_logger().error(f"service not available: {args.service}")
            return 1
        scene = PlanningScene()
        scene.is_diff = True
        for object_id in object_ids:
            obj = CollisionObject()
            obj.id = object_id
            obj.header.frame_id = args.frame_id
            obj.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(obj)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout_sec)
        if not future.done() or future.result() is None:
            node.get_logger().error("ApplyPlanningScene timed out")
            return 2
        if not future.result().success:
            node.get_logger().error("ApplyPlanningScene returned success=false")
            return 3
        node.get_logger().info("Removed collision objects: " + ", ".join(object_ids))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
