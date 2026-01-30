import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from pymodbus.client import ModbusSerialClient
from treadmill_interfaces.msg import TreadmillStatus
import glob
import time

# Definitions
# CPM = CFW-11 Programing Manual
# SCM = CFW-11 Serial Communication Manual

# temporary aproximation values for conversion
RPM_TO_MPS = 0.0108  # Conversion factor from RPM to meters per second
RPM_TO_MPS_INTERCEPT = -0.0056  # should not be nessasary

param_dict = {
    "speed_ref": 1,  # Reference speed (see CPM 16-1)
    "motor_speed": 2,  # Actual motor speed,
    "motor_current": 3,  # motor current
    "status": 6,  # Motor status
    "motor_voltage": 7,  # motor voltage
    "motor_torque": 9,  # motor torque
    "alarm_code": 48,  # alarm code (see CPM 16-9)
    "fault_code": 49,  # fault code (see CPM 16-9)
    "accel_time": 100,  # time to accelerate to full speed in seconds
    "decel_time": 101,  # time to decelerate to full stop in seconds
    "accel_time_2": 102,
    "decel_time_2": 103,
    "logic_status": 680,  # logic status see (SCM 13)
    "serial_control_word": 682,  # control word for commands from usb
    "serial_speed_ctrl": 683,  # speed control from usb
}


def get_fault(code, description_only=False):
    """
    Returns information for inverter Faults (F###).
    """
    faults = {
        6: {
            "desc": "Imbalance or Input Phase Loss",
            "causes": "Mains voltage imbalance too high; phase missing.",
        },
        21: {
            "desc": "DC Link Undervoltage",
            "causes": "Input voltage too low; Phase loss.",
        },
        22: {
            "desc": "DC Link Overvoltage",
            "causes": "Inertia too high; Decel time too short.",
        },
        30: {"desc": "Power Module U Fault", "causes": "Short-circuit U-V or U-W."},
        34: {"desc": "Power Module V Fault", "causes": "Short-circuit V-U or V-W."},
        38: {"desc": "Power Module W Fault", "causes": "Short-circuit W-U or W-V."},
        42: {"desc": "DB IGBT Fault", "causes": "Braking resistor short."},
        48: {"desc": "IGBT Overload Fault", "causes": "High current at output."},
        51: {"desc": "IGBT Overtemperature", "causes": "High temp on IGBTs."},
        71: {
            "desc": "Output Overcurrent",
            "causes": "Excessive load inertia; Accel time too short.",
        },
        72: {"desc": "Motor Overload", "causes": "Motor shaft load is excessive."},
        74: {"desc": "Ground Fault", "causes": "Short circuit to ground."},
        78: {"desc": "Motor Overtemperature", "causes": "Excessive load/duty cycle."},
        156: {"desc": "Undertemperature", "causes": "Air temp <= -30 °C."},
        185: {
            "desc": "Pre-charge Contactor Fault",
            "causes": "Open command fuse; Contactor defect.",
        },
    }

    data = faults.get(code)
    if not data:
        return f"Fault F{code:03d} not found in database."

    if description_only:
        return data["desc"]

    return (
        f"FAULT F{code:03d}\n"
        f"Description: {data['desc']}\n"
        f"Possible Causes: {data['causes']}"
    )


def get_alarm(code, description_only=False):
    """
    Returns information for inverter Alarms (A###).
    """
    alarms = {
        46: {"desc": "High Load on Motor", "causes": "High load."},
        47: {"desc": "IGBT Overload Alarm", "causes": "High current."},
        50: {"desc": "IGBT High Temperature", "causes": "High ambient temp."},
        88: {"desc": "Communication Lost", "causes": "Loose cable."},
        90: {"desc": "External Alarm", "causes": "Digital input open."},
        110: {"desc": "High Motor Temperature", "causes": "Excessive shaft load."},
        128: {"desc": "Timeout for Serial Communication", "causes": "Timeout."},
        152: {"desc": "Internal Air High Temperature", "causes": "Fan defective."},
        177: {"desc": "Fan Replacement", "causes": "Fan hours exceeded."},
        702: {"desc": "Inverter Disabled", "causes": "General Enable not active."},
    }

    data = alarms.get(code)
    if not data:
        return f"Alarm A{code:03d} not found in database."

    if description_only:
        return data["desc"]

    return (
        f"ALARM A{code:03d}\n"
        f"Description: {data['desc']}\n"
        f"Possible Causes: {data['causes']}"
    )


class Treadmill(Node):
    def __init__(self):
        super().__init__("treadmill_node")

        # === ROS 2 Parameters ===
        self.declare_parameter("serial_port", "")

        # === Setup Modbus RTU connection ===
        # Use parameter if provided, otherwise auto-detect
        port_param = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )

        if port_param:
            port = port_param
            self.get_logger().info(f"Using configured serial port: {port}")
        else:
            self.get_logger().info(
                "No serial port configured. Attempting to auto-detect..."
            )
            port = self._find_serial_port()

        if not port:
            self.get_logger().error("No matching USB serial device found.")
            # We exit constructor, but Node spin might still happen.
            # Ideally handled by logic checks later or raising exception.
            self.client = None
            return

        # Set up Modbus RTU connection
        self.client = ModbusSerialClient(
            port=port, baudrate=9600, parity="E", stopbits=1, bytesize=8, timeout=1
        )

        if not self.client.connect():
            self.get_logger().error(f"Failed to connect to CFW-11 on {port}")
            self.client = None
            return
        else:
            self.get_logger().info(f"Connected to CFW-11 on {port}")

        self.on_start()

        # === ROS 2 Subscribers ===
        self.create_subscription(
            String, "treadmill/special_cmd", self.special_cmd_callback, 10
        )
        self.create_subscription(
            Float32, "treadmill/cmd_speed_mps", self.set_speed_callback, 10
        )

        # === ROS 2 publishers ===
        self.pub_status = self.create_publisher(TreadmillStatus, "treadmill/status", 10)

        # Internal State Variables
        self.set_start_stop = False
        self.set_on_off = False
        self.set_direction = True
        self.set_jog = False
        self.set_loc_rem = True
        self.set_accel_profile = True
        self.set_quick_stop = False
        self.set_fault_reset = False

        # Speed tracking
        self.set_speed_rpm = 0.0

        # Timer to poll CFW-11 parameters
        self.create_timer(0.1, self.update_loop)

    def _find_serial_port(self):
        """
        Tries to find the serial port automatically.
        Prioritizes known IDs, then falls back to generic USB/ACM ports.
        """
        # 1. Try specific persistent IDs (more reliable if hardware is constant)
        for path in glob.glob("/dev/serial/by-id/*"):
            if "1a86" in path or "QinHeng" in path or "USB_Single_Serial" in path:
                return path

        # 2. Fallback: Return the first available USB or ACM serial device
        generic_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if generic_ports:
            return generic_ports[0]

        return None

    def read_param(self, param_name, type=float):
        """Read a parameter from the CFW-11 using its name."""
        if not self.client:
            return 0

        if param_name not in param_dict:
            raise ValueError(f"Unknown parameter name: {param_name}")
        param = param_dict.get(param_name)

        param_value = self.client.read_holding_registers(
            address=param, count=1, slave=1
        )
        if not param_value.isError():
            if type == int:
                return int(param_value.registers[0])
            else:
                return float(param_value.registers[0])
        return 0

    def special_cmd_callback(self, msg):
        string = msg.data

        if string == "go":
            self.set_start_stop = True
        elif string == "stop":
            self.set_start_stop = False
        elif string == "power_on":
            self.set_on_off = True
        elif string == "power_off":
            self.set_on_off = False
        elif string == "quick_stop":
            self.set_quick_stop = True
        elif string == "reset_fault":
            self.set_fault_reset = True
        elif string == "set_accel_profile_1":
            self.set_accel_profile = False
        elif string == "set_accel_profile_2":
            self.set_accel_profile = True
        else:
            self.get_logger().warn(f"Unknown command: {string}")

        self._send_special_command()

    def _send_special_command(self):
        if not self.client:
            return

        # Get control word as integer
        val = self.build_control_word()

        # Write register expects an integer, not a hex string
        self.client.write_register(address=682, value=val, slave=1)

        # Clean up momentary flags
        if self.set_quick_stop:
            self.set_quick_stop = False
        if self.set_fault_reset:
            self.set_fault_reset = False

    def build_control_word(self) -> int:
        """
        Calculates the integer value of the 16-bit control word.
        Uses self.set_... variables.
        """
        control_word = 0

        # Fixed variable names to match __init__ definitions
        if self.set_start_stop:
            control_word |= 1 << 0
        if self.set_on_off:  # General enabling
            control_word |= 1 << 1
        if self.set_direction:
            control_word |= 1 << 2
        if self.set_jog:
            control_word |= 1 << 3
        if self.set_loc_rem:
            control_word |= 1 << 4
        if self.set_accel_profile:  # Second ramp
            control_word |= 1 << 5
        if self.set_quick_stop:
            control_word |= 1 << 6
        if self.set_fault_reset:
            control_word |= 1 << 7

        return control_word

    def _get_speed(self):
        rpm = self.read_param("motor_speed")
        speed_mps = self._convert_rpm_to_speed(rpm)
        return speed_mps

    def _convert_speed_to_rpm(self, speed_mps):
        return speed_mps / RPM_TO_MPS

    def _convert_rpm_to_speed(self, rpm):
        return rpm * RPM_TO_MPS

    def set_speed_callback(self, msg):
        if not self.client:
            return

        speed = msg.data

        # Handle direction based on sign
        if speed < 0:
            speed = abs(speed)
            if self.set_direction:
                self.set_direction = False
                self._send_special_command()
        elif speed > 0:
            if not self.set_direction:
                self.set_direction = True
                self._send_special_command()

        # convert speed to rpm (Fixed function name)
        raw_rpm = self._convert_speed_to_rpm(speed)

        synchronous_speed = 1800
        max_resolution = 8192
        # get in 13 bit resolution
        rpm_cmd = int(raw_rpm * max_resolution / synchronous_speed)

        self.client.write_register(address=683, value=rpm_cmd, slave=1)
        self.set_speed_rpm = raw_rpm

    def update_loop(self):
        if not self.client:
            return

        # get current speed in mps (Fixed method name)
        speed_mps = self._get_speed()

        # get logic status
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
                present_fault = self.read_param("fault_code", type=int)
                fault_info = get_fault(present_fault)
                self.get_logger().error(fault_info)
                error_code = f"F{present_fault}"

            if alarm:
                present_alarm = self.read_param("alarm_code", type=int)
                alarm_info = get_alarm(present_alarm)
                self.get_logger().warn(alarm_info)
                error_code = f"A{present_alarm}"

        direction_sign = 1 if direction else -1

        # Publish treadmill status
        ms = TreadmillStatus()
        ms.speed_mps = speed_mps * direction_sign
        ms.error = error_code
        self.pub_status.publish(ms)

    def on_start(self):
        if not self.client:
            return
        # set control to vector control mode (just in case)
        self.client.write_register(address=202, value=4, slave=1)


def main(args=None):
    rclpy.init(args=args)

    node = Treadmill()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
