# Role: Select simple heuristic grasp/pre-grasp poses from 3D detections.

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Protocol

from geometry_msgs.msg import PoseStamped


@dataclass
class HeuristicGraspResult:
    """Result of a simple heuristic grasp planner."""

    class_name: str
    confidence: float
    pose: PoseStamped
    method: str


class GraspSelectorInterface(Protocol):
    """Interface that can later be implemented by AnyGrasp or another planner."""

    def compute_pregrasp_pose(
        self,
        detection: Dict[str, Any],
        z_offset: float,
        orientation_xyzw: List[float],
        frame_id: str,
    ) -> HeuristicGraspResult:
        ...


class HeuristicGraspSelector:
    """Fixed-orientation grasp selector for cup/lid pre-grasp testing."""

    def select_target(
        self,
        detections: List[Dict[str, Any]],
        target_class: str,
        min_confidence: float = 0.0,
    ) -> Optional[Dict[str, Any]]:
        candidates = [
            det
            for det in detections
            if det.get("class_name") == target_class
            and float(det.get("confidence", 0.0)) >= min_confidence
        ]
        if not candidates:
            return None

        return max(candidates, key=lambda det: float(det.get("confidence", 0.0)))

    def compute_pregrasp_pose(
        self,
        detection: Dict[str, Any],
        z_offset: float,
        orientation_xyzw: List[float],
        frame_id: str,
    ) -> HeuristicGraspResult:
        if "robot_xyz" not in detection:
            raise ValueError("3D detection must include robot_xyz.")
        if len(orientation_xyzw) != 4:
            raise ValueError("orientation_xyzw must have 4 values.")

        x, y, z = [float(v) for v in detection["robot_xyz"]]
        qx, qy, qz, qw = [float(v) for v in orientation_xyzw]

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z + float(z_offset)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return HeuristicGraspResult(
            class_name=str(detection.get("class_name", "unknown")),
            confidence=float(detection.get("confidence", 0.0)),
            pose=pose,
            method="bbox_center_depth_fixed_orientation",
        )
