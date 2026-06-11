#!/usr/bin/env bash
# 두 T_gripper2camera 캘리브레이션을 TF로 동시 퍼블리시.
# RViz에서 link_6 기준으로 두 camera frame 위치를 비교.
#
# 사용법:
#   source /home/ssu/Azas/install/local_setup.bash
#   bash tools/run/compare_hand_eye_in_rviz.sh
#
# RViz에서 확인:
#   - Fixed Frame: base_link
#   - Add > TF 체크
#   - camera_color_optical_frame_may20  (azas_perception, 5월20일)
#   - camera_color_optical_frame_may15  (dsr_practice, 5월15일)
#   둘을 link_6와 비교하면 카메라 장착 위치 차이를 직접 확인 가능.

set -e

echo "[compare_hand_eye] May20 (azas_perception): xyz=[0.0340, 0.0572, 0.0108]"
echo "[compare_hand_eye] May15 (dsr_practice)   : xyz=[0.0305, 0.0731, 0.0359]"
echo ""
echo "[compare_hand_eye] TF publisher 2개 백그라운드 실행 중..."

ros2 run tf2_ros static_transform_publisher \
  --x 0.0340 --y 0.0572 --z 0.0108 \
  --qx 0.0020 --qy 0.0031 --qz 1.0000 --qw -0.0033 \
  --frame-id link_6 \
  --child-frame-id camera_color_optical_frame_may20 &
PID1=$!

ros2 run tf2_ros static_transform_publisher \
  --x 0.0305 --y 0.0731 --z 0.0359 \
  --qx 0.0089 --qy 0.0050 --qz 0.9999 --qw -0.0013 \
  --frame-id link_6 \
  --child-frame-id camera_color_optical_frame_may15 &
PID2=$!

echo "[compare_hand_eye] PID $PID1 = May20, PID $PID2 = May15"
echo "[compare_hand_eye] RViz를 열고 TF를 추가하세요:"
echo "    rviz2 &"
echo ""
echo "  종료: Ctrl+C"

trap "kill $PID1 $PID2 2>/dev/null; echo 'stopped.'" EXIT
wait
