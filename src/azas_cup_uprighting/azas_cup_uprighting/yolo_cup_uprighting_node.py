
#!/usr/bin/env python3
"""
쓰러진 컵을 인식하고 보정된 오프셋으로 똑바로 세우는(Uprighting) 시나리오 노드.
"""

import time
import numpy as np
import cv2  

from . import _config as cfg
from ._base_node import BaseMoveItPickNode, run_node
from ._perception import calculate_cup_orientation, is_top_pointing_towards_theta
from ._motion import get_gripper_pose_by_cup
from scipy.spatial.transform import Rotation as R

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose



CUP_LENGTH_M = 0.12  
CUP_DIAMETER_M = 0.072 
CUP_RADIUS_M   = CUP_DIAMETER_M / 2.0 


class YoloCupUprightingNode(BaseMoveItPickNode):
    NODE_NAME        = "yolo_cup_uprighting_node"
    MOVEIT_NODE_NAME = "yolo_cup_uprighting_py"
    WINDOW_NAME      = "Cup Uprighting"

    def __init__(self):
        super().__init__()
        self.setup_safety_environment()

    def setup_safety_environment(self):
        log = self.get_logger()
        log.info("🚧 [안전망] YAML 기반 안전 환경(Keep-out Zone) 구축을 시작합니다...")

        pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        time.sleep(1.0)

       
        if cfg.SAFETY_CFG and 'motion' in cfg.SAFETY_CFG:
            bounds = cfg.SAFETY_CFG['motion']['workspace_bounds_m']
            
            self.arm.set_workspace(
                min_x=bounds['x_min'], min_y=bounds['y_min'], min_z=bounds['z_min'],
                max_x=bounds['x_max'], max_y=bounds['y_max'], max_z=bounds['z_max']
            )
            log.info(f"-> 작업 영역 동적 제한 완료 (Z_min: {bounds['z_min']}m)")
        else:
            log.warn("-> safety.yaml을 찾을 수 없어 기본 작업 영역 제한을 건너뜁니다.")
        

       
        if cfg.DISPENSER_CFG and 'estimated_collision_objects' in cfg.DISPENSER_CFG:
            
            disp_data = cfg.DISPENSER_CFG['estimated_collision_objects']['dispenser_combined_body_box']

            dispenser = CollisionObject()
            dispenser.header.frame_id = disp_data.get('frame_id', 'base_link')
            dispenser.id = "dispenser_combined_body_box"
            dispenser.operation = CollisionObject.ADD

            disp_box = SolidPrimitive()
            disp_box.type = SolidPrimitive.BOX
     
            disp_box.dimensions = disp_data['size_xyz_m']

            disp_pose = Pose()
            disp_pose.position.x = disp_data['center_xyz_m'][0]
            disp_pose.position.y = disp_data['center_xyz_m'][1]
            disp_pose.position.z = disp_data['center_xyz_m'][2]
            
            disp_pose.orientation.x = disp_data['orientation_xyzw'][0]
            disp_pose.orientation.y = disp_data['orientation_xyzw'][1]
            disp_pose.orientation.z = disp_data['orientation_xyzw'][2]
            disp_pose.orientation.w = disp_data['orientation_xyzw'][3]

            dispenser.primitives.append(disp_box)
            dispenser.primitive_poses.append(disp_pose)
            
            pub.publish(dispenser)
            log.info("-> YAML 기반 디스펜서 장애물 동적 등록 완료!")
        else:
            log.warn("-> 디스펜서 설정 파일을 찾을 수 없어 장애물 등록을 건너뜁니다.")

    
    def _select_target(self, detections):
        """
        현재 YOLO 모델의 실제 클래스 이름('cup')을 찾아 신뢰도가 가장 높은 객체를 선택
        """
        if not detections:
            return None
            
        target_candidates = [d for d in detections if d["cls_name"] == "cup"] 
        
        if not target_candidates:
            return None
            
        return max(target_candidates, key=lambda d: d["conf"])
    
    
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        vis = super()._draw_detections(frame)  # 기본 bbox 그리기

        for det in self._detections:
            if det["cls_name"] != "cup":
                continue

            x1, y1, x2, y2 = det["box"]
            cx, cy = det["cx"], det["cy"]

            # 1. 컵 주축 각도 계산
            theta = calculate_cup_orientation(self.depth_image, det["box"], frame)

            # 2. 입구 방향 판별
            is_top = is_top_pointing_towards_theta(frame, det["box"], theta)

            # 입구 방향이 theta 방향이면 그대로, 아니면 반전
            top_theta = theta if is_top else theta + np.pi

            # 3. 컵 주축 선 그리기 (흰색 양방향)
            length = max(x2 - x1, y2 - y1) // 2
            dx = int(np.cos(theta) * length)
            dy = int(np.sin(theta) * length)
            cv2.line(vis, (cx - dx, cy - dy), (cx + dx, cy + dy), (255, 255, 255), 2)

            # 4. 입구 방향 화살표 (초록), 바닥 방향 화살표 (파랑)
            top_dx = int(np.cos(top_theta) * length)
            top_dy = int(np.sin(top_theta) * length)
            bot_dx = int(np.cos(top_theta + np.pi) * length)
            bot_dy = int(np.sin(top_theta + np.pi) * length)

            cv2.arrowedLine(vis, (cx, cy), (cx + top_dx, cy + top_dy),
                            (0, 255, 0), 3, tipLength=0.3)   # 초록 = 입구
            cv2.arrowedLine(vis, (cx, cy), (cx + bot_dx, cy + bot_dy),
                            (255, 100, 0), 2, tipLength=0.2)  # 파랑 = 바닥

            # 5. 레이블
            label = f"top={'YES' if is_top else 'NO'} theta={np.degrees(theta):.1f}deg"
            cv2.putText(vis, label, (x1, y2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return vis
    
        

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
            return

        base = self.pixel_to_base(target["cx"], target["cy"])
        if base is None:
            log.error("픽셀 -> 베이스 3D 좌표 변환 실패.")
            return
        bx, by, bz = base
        
      
        cup_theta = calculate_cup_orientation(self.depth_image, target["box"], frame)

       
        # ==========================================================
        
        is_top = is_top_pointing_towards_theta(frame, target["box"], cup_theta)
        
        if  not is_top:
            log.info("[VISION] 컵이 반대로 누워있습니다. 카메라 상향 유지를 위해 파지 방향을 180도 뒤집습니다.")
            cup_theta += np.pi  # 180도 회전
        else:
            log.info("[VISION] 컵이 정방향입니다. 기본 파지 방향을 유지합니다.")
        # ==========================================================

        self.picking = True
        try:
            self._pick_and_straighten(bx, by, bz, cup_theta)
        finally:
            self.picking = False

       


    def _pick_and_straighten(self, bx, by, bz, cup_theta):
        log = self.get_logger()
        
        target_ori = get_gripper_pose_by_cup(cup_theta)

     
        TABLE_Z = 0.0 
        floor_z = TABLE_Z


        Z_OFFSET = cfg.Z_OFFSET  # 0.20m (20cm)

        PICK_CLEARANCE = 0.02 
        
        pick_z = floor_z + CUP_RADIUS_M + Z_OFFSET + PICK_CLEARANCE
        z_min = cfg.SAFETY_CFG['motion']['workspace_bounds_m']['z_min']
        place_z = max(floor_z + (CUP_LENGTH_M / 2.0), z_min)
        
        safe_z = floor_z + 0.25 + Z_OFFSET
        
        log.info(f"== 컵 구출 시퀀스 준비 (각도: {np.degrees(cup_theta):.1f}도) ==")
          


        log.info("[1-1] 상공 진입 (Z=25cm)")
        
        arm_component = self.robot.get_planning_component("manipulator")
        arm_component.set_start_state_to_current_state()
        current_state = arm_component.get_start_state()
        
        # 'link_6' 끝단의 현재 공간 좌표와 방향(Quaternion) 추출
        current_pose = current_state.get_pose("link_6") 
        
        current_ori = {
            "x": current_pose.orientation.x,
            "y": current_pose.orientation.y,
            "z": current_pose.orientation.z,
            "w": current_pose.orientation.w
        }
        
        # 추출한 현재 방향(current_ori)을 유지하면서 Z축만 상공으로 이동
        self.plan_pose(bx, by, safe_z, current_ori)
        time.sleep(1.0)


        log.info("[1-2] 상공에서 컵 방향으로 정렬")
        self.plan_pose(bx, by, safe_z, target_ori)
        time.sleep(1.0)

        log.info("[2] 컵 집기 시작")
        self.plan_pose(bx, by, pick_z, target_ori)
        self.gripper.close_gripper()
        log.info("[2] 컵 집기 완료")
        time.sleep(1.0)

        log.info("[3] Lift Up (다시 바닥 기준 25cm 상공으로 리프트업)")
        self.plan_pose(bx, by, safe_z, target_ori)
        time.sleep(1.0)

        # 직립화 실행 (항상 카메라가 위를 향하는 Roll=90 고정)
        log.info("[4] 컵 직립화 궤적 탐색 (카메라 상향 고정)...")

        dx = (CUP_LENGTH_M / 2.0) * np.cos(cup_theta)
        dy = (CUP_LENGTH_M / 2.0) * np.sin(cup_theta)
        place_x = bx - dx
        place_y = by - dy
        


        # 무조건 카메라가 위를 보는 자세(Roll=90) 쿼터니언 생성
        target_roll = 90
        quat_target = R.from_euler('xyz', [target_roll, 0, np.degrees(cup_theta)], degrees=True).as_quat()
        ori_target = {"x": float(quat_target[0]), "y": float(quat_target[1]), "z": float(quat_target[2]), "w": float(quat_target[3])}

        log.info("-> 카메라 상향(Roll=90) 궤적 플래닝 시도 중...")
        success = self.plan_pose(place_x, place_y, place_z + 0.15, ori_target)

        if success:
            log.info("=> 카메라 상향 직립화 궤적 채택 성공!")
            best_ori = ori_target
        else:
            log.error("=> 치명적 오류: 관절 한계로 인해 직립화 궤적 생성에 실패했습니다.")
            return 

        log.info("[4-1] 공중에서 컵 수직 정렬 완료")
        
        log.info(f"[4-2] Z-Height Adjustment (Z: {place_z:.3f})")
        self.plan_pose(place_x, place_y, place_z + 0.02, best_ori)

        
        log.info("[5] Place & Release")
        self.plan_pose(place_x, place_y, place_z, best_ori)
        self.gripper.open_gripper()
        time.sleep(1.0)
        
        log.info("[6] Retract")
        self.plan_pose(place_x, place_y, place_z + 0.15, best_ori)
        

        log.info("[7] 홈 위치로 복귀")
        if self.go_home_pose():
            log.info("=> 홈 복귀 완료")
        else:
            log.error("=> 홈 복귀 실패")

        

def main(args=None):
    run_node(YoloCupUprightingNode)


if __name__ == "__main__":
    main()
