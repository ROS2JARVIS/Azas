"""Low-level Modbus TCP driver for OnRobot RG2/RG6 grippers."""

try:
    from pymodbus.client import ModbusTcpClient as ModbusClient
except ImportError:  # pragma: no cover - pymodbus 2.x fallback
    from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class RG:
    """Small wrapper around the OnRobot RG Modbus register interface."""

    UNIT_ID = 65

    def __init__(self, gripper, ip, port):
        if gripper not in ["rg2", "rg6"]:
            raise ValueError("Please specify either rg2 or rg6.")

        self.client = ModbusClient(
            ip,
            port=int(port),
            stopbits=1,
            bytesize=8,
            parity="E",
            baudrate=115200,
            timeout=1,
        )
        self.gripper = gripper
        if self.gripper == "rg2":
            self.max_width = 1100
            self.max_force = 400
        else:
            self.max_width = 1600
            self.max_force = 1200
        self.open_connection()

    def _read_holding_registers(self, address, count):
        try:
            return self.client.read_holding_registers(
                address=address, count=count, slave=self.UNIT_ID
            )
        except TypeError:
            return self.client.read_holding_registers(
                address=address, count=count, unit=self.UNIT_ID
            )

    def _write_register(self, address, value):
        try:
            return self.client.write_register(
                address=address, value=value, slave=self.UNIT_ID
            )
        except TypeError:
            return self.client.write_register(
                address=address, value=value, unit=self.UNIT_ID
            )

    def _write_registers(self, address, values):
        try:
            return self.client.write_registers(
                address=address, values=values, slave=self.UNIT_ID
            )
        except TypeError:
            return self.client.write_registers(
                address=address, values=values, unit=self.UNIT_ID
            )

    def open_connection(self):
        """Open the TCP connection with the gripper."""
        connected = self.client.connect()
        if connected is False:
            raise ConnectionError("Failed to connect to OnRobot gripper")

    def close_connection(self):
        """Close the TCP connection with the gripper."""
        self.client.close()

    def get_fingertip_offset(self):
        """Read the current fingertip offset in millimeters."""
        result = self._read_holding_registers(address=258, count=1)
        return result.registers[0] / 10.0

    def get_width(self):
        """Read current width between gripper fingers in millimeters."""
        result = self._read_holding_registers(address=267, count=1)
        return result.registers[0] / 10.0

    def get_status(self):
        """Read the current device status as seven boolean-like flags."""
        result = self._read_holding_registers(address=268, count=1)
        status = format(result.registers[0], "016b")
        return [int(status[-idx]) for idx in range(1, 8)]

    def get_width_with_offset(self):
        """Read current width with the configured fingertip offset included."""
        result = self._read_holding_registers(address=275, count=1)
        return result.registers[0] / 10.0

    def set_control_mode(self, command):
        """Set the gripper control mode register."""
        return self._write_register(address=2, value=command)

    def set_target_force(self, force_val):
        """Write target force in 1/10 newton units."""
        force_val = max(0, min(int(force_val), self.max_force))
        return self._write_register(address=0, value=force_val)

    def set_target_width(self, width_val):
        """Write target width in 1/10 millimeter units."""
        width_val = max(0, min(int(width_val), self.max_width))
        return self._write_register(address=1, value=width_val)

    def close_gripper(self, force_val=400):
        """Close the gripper."""
        return self.move_gripper(0, force_val)

    def open_gripper(self, force_val=400):
        """Open the gripper to its maximum width."""
        return self.move_gripper(self.max_width, force_val)

    def move_gripper(self, width_val, force_val=400):
        """Move gripper to width and force targets in register units."""
        force_val = max(0, min(int(force_val), self.max_force))
        width_val = max(0, min(int(width_val), self.max_width))
        return self._write_registers(address=0, values=[force_val, width_val, 16])
