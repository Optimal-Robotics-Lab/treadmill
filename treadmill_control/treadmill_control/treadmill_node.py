import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from pymodbus.client import ModbusSerialClient
from treadmill_interfaces.msg import TreadmillStatus
import glob
from math import pi

"""
Reference Definitions
PM = CFW-11 Programing Manual
SCM = CFW-11 Serial Communication Manual

Both references are found in the docs folder
"""


radius = 0.177  # roller radius in meters
ratio = 56 / 80  # gear ratio


def rpm_to_mps(rpm):
    return 2 * pi * radius * (rpm / 60) * ratio


def mps_to_rpm(mps):
    return round((mps / (2 * pi * radius * ratio)) * 60, 0)


class Treadmill(Node):
    def __init__(self):
        super().__init__("treadmill_control")

        port = self._find_serial_port()
        if not port:
            self.get_logger().error("No matching USB serial device found.")
            return

        # Set up Modbus RTU connection
        self.client = ModbusSerialClient(
            port=port, baudrate=9600, parity="E", stopbits=1, bytesize=8, timeout=1
        )
        if not self.client.connect():
            self.get_logger().error(f"Failed to connect to CFW-11 on {port}")
            return
        else:
            self.get_logger().info(f"Connected to CFW-11 on {port}")

        # === ROS 2 Publishers and Subscribers ===

        self.pub_status = self.create_publisher(TreadmillStatus, "treadmill/status", 10)

        self.create_subscription(
            Float32, "treadmill/cmd_speed_mps", self.set_speed_callback, 10
        )
        self.create_subscription(
            String, "treadmill/special_cmd", self.special_cmd_callback, 10
        )

        # update loop timer to periaodically read status and publish
        self.create_timer(0.1, self.update_loop)

        # == special_cmd instance variables ==
        self.start = False  # treadmill moving or not
        self.on = False  # power on/off
        self.direction = True  # True for forward, False for reverse
        self.jog = False  # False for normal operation, True for jog mode
        self.local = False  # False for remote mode, True for local mode
        self.ramp_profile = (
            False  # False = acceleration profile 1 ,True = accel profile 2
        )
        self.quick_stop = False  # False for not actived, True for quick stop activated
        self.fault_reset = (
            False  # False for normal operation, True for sending fault reset command
        )

    def _find_serial_port(self):
        """
        Tries to find the serial port automatically.
        Prioritizes known IDs, then falls back to generic USB/ACM ports.
        """
        for path in glob.glob("/dev/serial/by-id/*"):
            if "1a86" in path or "QinHeng" in path or "USB_Single_Serial" in path:
                return path

        # Fallback: Return the first available USB or ACM serial device
        generic_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if generic_ports:
            return generic_ports[0]

        return None

    def set_speed_callback(self, msg):
        # for more details see SCM pg 16
        speed_mps = msg.data
        rpm = mps_to_rpm(speed_mps)
        synchronous_speed = 1800
        max_resolution = 8192

        value = int(rpm * max_resolution / synchronous_speed)

        # untested code to handle negative values
        if value < 0:
            value = value & 0xFFFF

        value = max(0, min(value, 65535))

        self.client.write_register(address=683, value=value, slave=1)
        self.get_logger().info(f"Set speed to {rpm:.2f} RPM")

    def update_loop(self):
        # for more details see PM 16-2
        read_rpm = self.client.read_holding_registers(address=2, count=1, slave=1)
        if not read_rpm.isError():
            rpm = float(read_rpm.registers[0])
            speed_mps = rpm_to_mps(rpm)

        else:
            speed_mps = -999999

        # for more details see SCM pg. 13-14
        logic_msg = self.client.read_holding_registers(address=680, count=1, slave=1)

        # Check for error first
        if logic_msg.isError():
            self.get_logger().warn("Failed to read logic status")
            return

        logic_val = int(logic_msg.registers[0])

        # Bit parsing
        faulted = bool(logic_val & (1 << 15))
        undervoltage = bool(logic_val & (1 << 13))
        direction = bool(logic_val & (1 << 10))
        power_on = bool(logic_val & (1 << 9))
        running = bool(logic_val & (1 << 8))
        alarm = bool(logic_val & (1 << 7))
        accel_profile = bool(logic_val & (1 << 5))
        quick_stop = bool(logic_val & (1 << 4))

        error_code = ""

        # if fault or alarm present , read and publish info
        if faulted or alarm:
            if faulted:
                read_fault = self.client.read_holding_registers(
                    address=49, count=1, slave=1
                )
                if not read_fault.isError():
                    present_fault = float(read_fault.registers[0])
                else:
                    present_fault = "Cannot read fault code"
                error_code = f"F{present_fault}"

            if alarm:
                read_alarm = self.client.read_holding_registers(
                    address=48, count=1, slave=1
                )
                if not read_alarm.isError():
                    present_alarm = float(read_alarm.registers[0])
                else:
                    present_alarm = "Cannot read alarm code"
                error_code = f"A{present_alarm}"

        read_status = self.client.read_holding_registers(address=6, count=1, slave=1)
        if not read_status.isError():
            status_raw = float(read_status.registers[0])
            status = str(status_raw)
        else:
            self.get_logger().warn("Failed to read status word")
            status = "failed to read status"

        # Publish treadmill status
        msg = TreadmillStatus()
        msg.speed_mps = speed_mps
        msg.error = error_code
        msg.status = status
        self.pub_status.publish(msg)

    def special_cmd_callback(self, msg):
        string = msg.data

        if string == "start":
            self.start = True
        elif string == "stop":
            self.start = False
        elif string == "on":
            self.on = True
        elif string == "off":
            self.on = False
        elif string == "quick_stop":
            self.quick_stop = True
        elif string == "reset_fault":
            self.fault_reset = True
        elif string == "set_accel_profile_1":
            self.ramp_profile = False
        elif string == "set_accel_profile_2":
            self.ramp_profile = True
        else:
            self.get_logger().warn(f"Unknown command: {string}")

        word = self.construct_control_word()

        self.client.write_register(address=682, value=word, slave=1)

        # Clean up momentary flags
        if self.quick_stop:
            self.quick_stop = False
        if self.fault_reset:
            self.fault_reset = False

    def construct_control_word(self):
        """
        Create the control word based on the current state of the command flags.
        for param 682 for details see
        """

        control_word = 0
        control_word |= self.start << 0
        control_word |= self.on << 1
        control_word |= self.direction << 2
        control_word |= self.jog << 3
        control_word |= self.local << 4
        control_word |= self.ramp_profile << 5
        control_word |= self.quick_stop << 6
        control_word |= self.fault_reset << 7

        return control_word


def main(args=None):
    rclpy.init(args=args)
    node = Treadmill()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
