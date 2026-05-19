#!/usr/bin/env python3
# rg2_gripper_node.py

import time

import rclpy

from .onrobot import RG  # 같은 패키지 내부의 onrobot.py 사용


GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502

# 그리퍼 폭 (raw 단위: 1/10 mm)
GRIPPER_OPEN_WIDTH = 1100  # 110.0 mm, RG2 최대 벌림
GRIPPER_CLOSE_WIDTH = 0    # 완전히 닫는 방향으로 이동하다가 grip_detected 시 정지
GRIPPER_FORCE = 200

STATUS_CHECK_INTERVAL = 0.05
GRIP_TIMEOUT_SEC = 8.0


def wait_until_not_busy(gripper, logger, timeout_sec=5.0):
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        status = gripper.get_status()
        busy = bool(status[0])
        if not busy:
            return True
        time.sleep(STATUS_CHECK_INTERVAL)

    logger.warning("그리퍼 동작 완료 대기 시간이 초과되었습니다.")
    return False


def close_until_grip_detected(gripper, logger):
    logger.info(
        f"그리퍼 CLOSE 시작 (목표 폭={GRIPPER_CLOSE_WIDTH}, 힘={GRIPPER_FORCE})"
    )
    gripper.move_gripper(width_val=GRIPPER_CLOSE_WIDTH, force_val=GRIPPER_FORCE)

    start_time = time.time()
    while time.time() - start_time < GRIP_TIMEOUT_SEC:
        status = gripper.get_status()
        busy = bool(status[0])
        grip_detected = bool(status[1])

        if grip_detected:
            logger.info("Grip detected - 컵을 잡았으므로 그리퍼를 정지합니다.")
            gripper.set_control_mode(8)
            return True

        if not busy:
            logger.warning("그리퍼가 끝까지 닫혔지만 grip_detected가 감지되지 않았습니다.")
            return False

        time.sleep(STATUS_CHECK_INTERVAL)

    logger.error("Grip 감지 대기 시간이 초과되었습니다. 그리퍼를 정지합니다.")
    gripper.set_control_mode(8)
    return False


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rg2_gripper_node")

    logger = node.get_logger()
    logger.info("=== RG2 Gripper Control Node 시작 ===")

    # ---- 그리퍼 연결 ----
    gripper = RG(
        gripper=GRIPPER_NAME,
        ip=TOOLCHARGER_IP,
        port=TOOLCHARGER_PORT,
    )
    time.sleep(0.5)  # 연결 안정화 대기
    logger.info("그리퍼와 연결됨")

    logger.info(f"그리퍼 최대 OPEN (폭={GRIPPER_OPEN_WIDTH})")
    gripper.move_gripper(width_val=GRIPPER_OPEN_WIDTH, force_val=GRIPPER_FORCE)
    wait_until_not_busy(gripper, logger)

    grip_detected = close_until_grip_detected(gripper, logger)
    if grip_detected:
        logger.info("Grip 성공 - 컵을 잡은 위치에서 정지했습니다.")
    else:
        logger.error("Grip 실패 - 컵을 잡지 못했거나 감지하지 못했습니다.")

    logger.info("=== RG2 Gripper Control Node 종료 ===")
    gripper.close_connection()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
