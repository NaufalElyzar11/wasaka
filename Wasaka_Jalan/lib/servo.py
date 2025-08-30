from dynamixel_sdk import *  # Dynamixel SDK

class DynamixelServo:
    def __init__(self, device_name="COM21", baudrate=1000000, protocol_version=1.0):
        # Control table addresses
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_GOAL_POSITION = 30
        self.LEN_GOAL_POSITION = 2
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.current_positions = {}

        # Initialize port and packet handler
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(protocol_version)

        # Open port and set baudrate
        if self.port_handler.openPort() and self.port_handler.setBaudRate(baudrate):
            print("Port opened successfully.")
        else:
            raise Exception("Failed to open port or set baudrate.")

        # Create GroupSyncWrite instance
        self.group_sync_write = GroupSyncWrite(
            self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION
        )

    def enable_torque(self, servo_ids):
        """Enable torque for specified servo IDs."""
        print("Servo Turned ON")
        for servo_id in servo_ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
            )

    def disable_torque(self, servo_ids):
        """Disable torque for specified servo IDs."""
        print("Servo Turned OFF")
        for servo_id in servo_ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
            )

    def write(self, servo_ids, positions):
        """
        Write goal positions to multiple servos if they are different from the current positions.
        :param servo_ids: List of servo IDs.
        :param positions: List of goal positions corresponding to servo IDs.
        """
        if len(servo_ids) != len(positions):
            raise ValueError("Length of servo_ids and positions must match.")

        any_changes = False

        # Add parameters for each servo only if the position has changed
        for i, servo_id in enumerate(servo_ids):
            position = positions[i]

            # Write only if position has changed
            if self.current_positions.get(servo_id) != position:
                param_goal_position = [DXL_LOBYTE(position), DXL_HIBYTE(position)]
                self.group_sync_write.addParam(servo_id, param_goal_position)
                self.current_positions[servo_id] = position
                any_changes = True

        # Send packet only if there are changes
        if any_changes:
            self.group_sync_write.txPacket()
            self.group_sync_write.clearParam()

    def close(self):
        """Disable torque and close the port."""
        self.port_handler.closePort()
        print("Port closed.")

# Example usage
if __name__ == "__main__":
    # Initialize the servo controller
    servo = DynamixelServo(device_name="COM21", baudrate=1000000)

    # Servo IDs and goal positions
    servo_ids = list(range(1, 19))  # Servo IDs from 1 to 18
    goal_positions = [512 for _ in range(18)]  # All servos to position 512

    try:
        # Enable torque
        servo.enable_torque(servo_ids)

        # Move servos
        servo.write(servo_ids, goal_positions)

    finally:
        # Disable torque and close
        servo.disable_torque(servo_ids)
        servo.close()
