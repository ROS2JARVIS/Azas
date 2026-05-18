# AnyGrasp Integration Review

This is the first integration plan for using an overhead RGB-D camera to produce grasp candidates for Azas.

## Candidate

- Repository: `https://github.com/CollaborativeRoboticsLab/anygrasp_ros`
- Upstream role: ROS 2 wrapper around AnyGrasp detection/tracking.
- Expected input: point cloud or RGB-D pipeline inside the AnyGrasp wrapper.
- Expected output: grasp poses as `geometry_msgs/PoseStamped[]`, plus RViz marker topics.
- Status: experimental review only. Do not connect to real robot motion until the safety gate below passes.

## Desired Azas Flow

```text
top RGB-D camera
-> aligned depth / point cloud
-> anygrasp_ros detection or tracking
-> azas_perception adapter publishes /azas/grasp_candidates
-> azas_task_manager selects a candidate with workspace and collision checks
-> azas_motion plans with MoveItPy
-> azas_gripper closes RG2
```

## Adapter Boundary

Azas should not let an external package command the robot directly. External grasp results must be converted into Azas messages first:

```text
geometry_msgs/PoseStamped[] from AnyGrasp
-> azas_interfaces/GraspCandidateArray
-> tf2 transform to base_link if needed
-> reject stale frame, low score, invalid orientation, workspace violation
```

## Review Workspace

Import only into a disposable workspace:

```bash
mkdir -p /tmp/azas_oss_review_experimental/src
cd /tmp/azas_oss_review_experimental
vcs import src < /home/ssu/Azas/dependencies/experimental_sources.repos
```

Do not copy the imported upstream source tree into `/home/ssu/Azas`.

## Safety Gate

Before any real M0609/RG2 execution:

1. Verify SDK license and permitted use.
2. Verify ROS distro compatibility with Humble or isolate the wrapper in a container/bridge.
3. Verify CUDA, PyTorch, and driver compatibility on the RTX 5080 host.
4. Verify camera optical frame to ROS frame convention.
5. Verify `camera_frame -> base_link` TF with AprilTag or hand-eye calibration.
6. Visualize candidate grasps in RViz without motion.
7. Run MoveIt fake execution only, with table, camera mount, dispenser, and cup collision objects.
8. Run low-speed real hardware only after fake execution and operator checklist pass.

## Known Risks

- AnyGrasp SDK requires license registration and model weights outside the Azas repo.
- The public ROS wrapper currently targets a newer ROS/Python/CUDA stack than Azas Humble, so a bridge/container may be needed.
- Transparent or reflective cups can degrade depth and point cloud quality.
- Grasp frames may not match RG2 `gripper_tcp`; an adapter transform is required.
- A valid grasp pose is not a valid robot motion. MoveIt and PlanningScene remain mandatory.
