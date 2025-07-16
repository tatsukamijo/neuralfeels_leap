#!/usr/bin/env python3
"""Enable torque for all connected Dynamixel motors."""
import logging
from leap_hand_utils.dynamixel_client import DynamixelClient

MOTOR_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
PORT = '/dev/ttyUSB4'
BAUDRATE = 3000000

logging.basicConfig(level=logging.INFO)


def main() -> None:
    """Enables torque for all motors in MOTOR_IDS."""
    try:
        client = DynamixelClient(MOTOR_IDS, PORT, BAUDRATE)
        if not client.is_connected:
            client.connect()
        client.set_torque_enabled(MOTOR_IDS, True)
        logging.info("Enabled torque for all motors: %s", MOTOR_IDS)
    except Exception as e:
        logging.error("Failed to enable torque. Function: main, Args: motor_ids=%s, port=%s, baudrate=%d, Error: %s", MOTOR_IDS, PORT, BAUDRATE, str(e))
        raise
    finally:
        if 'client' in locals() and client.is_connected:
            client.disconnect()


if __name__ == "__main__":
    main() 