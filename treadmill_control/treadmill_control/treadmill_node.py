import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from pymodbus.client import ModbusSerialClient
from treadmill_interfaces.msg import TreadmillStatus
import glob

# Definitions
# CPM = CFW-11 Programing Manual
# SCM = CFW-11 Serial Communication Manual


RPM_TO_MPS = ? # Conversion factor from RPM to meters per second


param_dict = {
    "speed_ref" : 1, # Reference speed (see CPM 16-1)
    "motor_speed": 2, # Actual motor speed,
    "motor_current": 3, # motor current
    "status": 6, # Motor status
    "motor_voltage": 7, # motor voltage
    "motor_torque" : 9, # motor torque
    "alarm_code": 48, # alarm code (see CPM 16-9)
    "fault_code": 49, # fault code (see CPM 16-9)
    "accel_time": 100, # time to accelerate to full speed in seconds
    "decel_time": 101, # time to decelerate to full stop in seconds
    "accel_time_2": 102,
    "decel_time_2": 103,
    "logic_status": 680, # logic status see (SCM 13)
    "serial_control_word": 682, # control word for commands from usb
    "serial_speed_ctrl": 683, # speed control from usb
}
def get_fault(code, description_only=False):
    """
    Returns information for inverter Faults (F###).
    :param code: Integer representing the fault code (e.g., 6 for F006).
    :param description_only: Boolean, if True returns only the description.
    """
    faults = {
        6: {
            "desc": "Imbalance or Input Phase Loss",
            "causes": "Mains voltage imbalance too high; phase missing at input; input voltage imbalance > 5%."
        },
        21: {
            "desc": "DC Link Undervoltage",
            "causes": "Input voltage too low; Phase loss; Pre-charge circuit failure; P0296 set too high."
        },
        22: {
            "desc": "DC Link Overvoltage",
            "causes": "Inertia of driven-load too high; Deceleration time too short; P0151/P0153/P0185 set too high."
        },
        30: {"desc": "Power Module U Fault", "causes": "IGBT desaturation; Short-circuit between phases U-V or U-W."},
        34: {"desc": "Power Module V Fault", "causes": "IGBT desaturation; Short-circuit between phases V-U or V-W."},
        38: {"desc": "Power Module W Fault", "causes": "IGBT desaturation; Short-circuit between phases W-U or W-V."},
        42: {"desc": "DB IGBT Fault", "causes": "Desaturation of Dynamic Braking IGBT; Short-circuit in braking resistor cables."},
        48: {"desc": "IGBT Overload Fault", "causes": "High current at inverter output; check switching frequency vs. load."},
        51: {"desc": "IGBT Overtemperature", "causes": "High temperature detected by NTC sensors on IGBTs."},
        71: {"desc": "Output Overcurrent", "causes": "Excessive load inertia; Acceleration time too short; P0135/P0169/P0170 too high."},
        72: {"desc": "Motor Overload", "causes": "P0156/P0157/P0158 settings too low; Motor shaft load is excessive."},
        74: {"desc": "Ground Fault", "causes": "Short circuit to ground in motor/cables; Motor cable capacitance too large."},
        78: {"desc": "Motor Overtemperature", "causes": "Excessive motor load/duty cycle; Ambient temp too high; PTC wiring issues."},
        156: {"desc": "Undertemperature", "causes": "Surrounding air temperature <= -30 °C (-22 °F)."},
        185: {"desc": "Pre-charge Contactor Fault", "causes": "Open command fuse; Phase loss in L1/R or L2/S; Contactor defect."}
    }

    data = faults.get(code)
    if not data:
        return f"Fault F{code:03d} not found in database."

    if description_only:
        return data["desc"]
    
    return (f"FAULT F{code:03d}\n"
            f"Description: {data['desc']}\n"
            f"Possible Causes: {data['causes']}")


def get_alarm(code, description_only=False):
    """
    Returns information for inverter Alarms (A###).
    :param code: Integer representing the alarm code (e.g., 46 for A046).
    :param description_only: Boolean, if True returns only the description.
    """
    alarms = {
        46: {
            "desc": "High Load on Motor",
            "causes": "Settings of P0156/P0157/P0158 too low; Motor shaft load excessive."
        },
        47: {
            "desc": "IGBT Overload Alarm",
            "causes": "High current at inverter output; Check switching frequency table."
        },
        50: {
            "desc": "IGBT High Temperature",
            "causes": "High ambient temp (> 45°C); Fan blocked/defective; Dirty heatsink."
        },
        88: {"desc": "Communication Lost", "causes": "Loose keypad cable; Electrical noise in installation."},
        90: {"desc": "External Alarm", "causes": "Digital input set to 'No External Alarm' is open/unwired."},
        110: {"desc": "High Motor Temperature", "causes": "Excessive shaft load; High ambient temp; PTC sensor wiring issues."},
        128: {"desc": "Timeout for Serial Communication", "causes": "No valid messages received within P0314 interval; Check wiring/grounding."},
        152: {"desc": "Internal Air High Temperature", "causes": "Ambient temp > 45°C; Internal fan defective."},
        177: {"desc": "Fan Replacement", "causes": "Heatsink fan has exceeded 50,000 operating hours."},
        702: {"desc": "Inverter Disabled", "causes": "General Enable command not active while SoftPLC is in Run mode."}
    }

    data = alarms.get(code)
    if not data:
        return f"Alarm A{code:03d} not found in database."

    if description_only:
        return data["desc"]

    return (f"ALARM A{code:03d}\n"
            f"Description: {data['desc']}\n"
            f"Possible Causes: {data['causes']}")


class Treadmill(Node):
    def __init__(self):
        super().__init__("treadmill_node")

        # === Setup Modbus RTU connection ===

        # Find USB device dynamically ( may need fix)
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

        self.on_start()


        # === ROS 2 Subscribers ===

        self.create_subscription(String, "treadmill/special_cmd", self.special_cmd_callback, 10)
        self.create_subscription(Float32, "treadmill/cmd_speed_mps", self.set_speed_callback, 10)

        # === ROS 2 publishers ===

        self.pub_status = self.create_publisher(TreadmillStatus, "treadmill/status", 10)
        
        
        # self.rpm = 0.0
        # self.speed_mps = 0.0
        # self.set_speed_rpm = 0.0
        
        # For more details on what the below varaibles are based on see 
        # the CFW-11 RS-232/RS-485 manual parameter P0682 or general programming manual
        #
        # self.direction = 1 # 1 is moving away from the wall , 0 is moving towards the wall
        # self.running = False # Start/Stop in manual
        # self.power_on = False # is the inverter power supply on or off (Enabled/Disables in manual)
        # self.quick_stop = False # is the quick stop activated 
        # self.accel_profile = 0 # is the accel profile 1 or 2, 0 = accel profile 1, 1 = accel profile 2
        #
        # # Diagnostic flags
        # self.faulted = False # is there a fault in the inverter
        # self.undervoltage = False # is there an undervoltage condition in the inverter
        # self.alarm = False # inverter alarm flag
        #
        # special command set variables 
        self.set_start_stop = False
        self.set_on_off = False
        self.set_direction = True
        self.set_jog = False
        self.set_loc_rem = True
        self.set_accel_profile = True
        self.set_quick_stop = False
        self.set_fault_reset = False

        # Timer to poll CFW-11 parameters
        self.create_timer(0.1, self.update_loop)


    # May need to be fixed carried from old version
    def _find_serial_port(self):
        for path in glob.glob('/dev/serial/by-id/*'):
            if '1a86' in path or 'QinHeng' in path or 'USB_Single_Serial' in path:
                return path
        return None


    def read_param(self, param_name, type=float):
        """Read a parameter from the CFW-11 using its name."""

        if param_name not in param_dict:
            raise ValueError(f"Unknown parameter name: {param_name}")
        param = param_dict.get(param_name)

        param_value = self.client.read_holding_registers(address=param, count=1, slave=1)
        if not param_value.isError():
            if type == int:
                return int(param_value.registers[0])
            else:
                return float(param_value.registers[0])

    def _read_logic_status(self):

        logic_msg =  self.client.read_holding_registers(address=680, count=1, slave=1)
        if not logic_msg.isError():
            return int(logic_msg.registers[0])

        # Bit 15 fault condition
        self.faulted = bool(logic_msg & (1 << 15))

        # Bit 13 Undervoltage
        self.undervoltage = bool(logic_msg & (1 << 13))

        # Bit 10 Speed Direction
        self.direction = bool(logic_msg & (1 << 10)) # 0 =

        # Bit 9 General Enabling active
        self.power_on = bool(logic_msg & (1 << 9))

        # Bit 8 Ramp Enabled (RUN)
        self.running = bool(logic_msg & (1 << 8))

        # Bit 7 Alarm 
        self.alarm = bool(logic_msg & (1 << 7))

        # Bit 5 Accel/Decel profile
        self.accel_profile = bool(logic_msg & (1 << 5)) # 0 = profile 1, 1 = profile 2

        # Bit 4 Quick stop active
        self.quick_stop = bool(logic_msg & (1 << 4))

        # === Curently unused bits ===
        
        # Bit 14 Manual/Automatic
        # pid_automatic = bool(logic_msg & (1 << 14)) # 0 = Manual, 1 = Automatic
        
        # Bit 12 LOC/REM
        # is_local = bool(logic_msg & (1 << 12)) # 0 = Local, 1 = Remote

        # Bit 11 JOG 
        # jog_active = bool(logic_msg & (1 << 11)) # 0 = No JOG, 1 = JOG active

        # Bit 6 In config mode 
        # in_config_mode = bool(logic_msg & (1 << 6))

    
       
    def special_cmd_callback(self, msg):
        string = msg.data


        if string == "start":
            self.set_start_stop = True
        elif string == "stop":
            self.set_start_stop = False
        elif string == "on":
            self.set_on_off = True
        elif string == "off":
         self.set_on_off = False
        elif string == "quick_stop":
            self.set_quick_stop = True
        elif string == "reset_fault":
            self.set_fault_reset = True
        elif string == "set_accel_profile_1":
            self.set_accel_profile = False
        elif string == "set_accel_profile_2":
            self.set_accel_profile = False
        elif string == "set_direction_forward":
            self.set_direction = True
        elif string == "set_direction_reverse":
            self.set_direction = False
        else: 
            print("Unknown command")

        # if this does not work try to send as binary
        msg = self.get_hex()

        self.client.write_register(address=682, value=msg, slave=1)

        # Clean up special commands to prevent repeated sending
        if self.set_quick_stop == True:
            self.set_quick_stop = False
        if self.set_fault_reset == True:
            self.set_fault_reset = False


    def build_control_word(self) -> int:
        """
        Calculates the integer value of the 16-bit control word.
        """
        control_word = 0
        
        if self.start_stop:
            control_word |= (1 << 0)
        if self.general_enabling:
            control_word |= (1 << 1)
        if self.direction:
            control_word |= (1 << 2)
        if self.jog:
            control_word |= (1 << 3)
        if self.loc_rem:
            control_word |= (1 << 4)
        if self.second_ramp:
            control_word |= (1 << 5)
        if self.quick_stop:
            control_word |= (1 << 6)
        if self.fault_reset:
            control_word |= (1 << 7)
        
        # Bits 8-15 are reserved (remain 0)
        return control_word
    
    def get_hex(self):
        return f"0x{self.build_control_word():04X}"

    def _get_speed(self):
        rpm = self.read_param("motor_speed")
        speed_mps = self._convert_rpm_to_speed(rpm)
        return speed_mps

    def _convert_speed_to_rpm(self, speed_mps):
        return speed_mps / RPM_TO_MPS

    def _convert_rpm_to_speed(self, rpm):
        return rpm * RPM_TO_MPS

    def set_speed_callback(self, msg):
        # for details see SCM page 14
        speed = msg.data
        # convert speed to rpm
        raw_rpm = self._speed_to_rpm(speed)
        synchronous_speed = 1800
        max_resolution = 8192
        # get in 13 bit resolution
        rpm = int(raw_rpm * max_resolution / synchronous_speed)
        self.client.write_register(address=683, value=rpm, slave=1)
        self.get_logger().info(f"Set speed to {raw_rpm:.2f} RPM")
        print(f"Set speed to {speed:.2f} RPM")
        self.set_speed_rpm = raw_rpm

    def update_loop(self):
        # get current speed in mps
        speed_mps = self.get_speed()

        # get logic status
        logic_msg =  self.client.read_holding_registers(address=680, count=1, slave=1)
        if not logic_msg.isError():
            return int(logic_msg.registers[0])

        # Bit 15 fault condition
        faulted = bool(logic_msg & (1 << 15))
        # Bit 13 Undervoltage
        undervoltage = bool(logic_msg & (1 << 13))
        # Bit 10 Speed Direction
        direction = bool(logic_msg & (1 << 10)) # 0 =
        # Bit 9 General Enabling active
        power_on = bool(logic_msg & (1 << 9))
        # Bit 8 Ramp Enabled (RUN)
        running = bool(logic_msg & (1 << 8))
        # Bit 7 Alarm 
        alarm = bool(logic_msg & (1 << 7))
        # Bit 5 Accel/Decel profile
        accel_profile = bool(logic_msg & (1 << 5)) # 0 = profile 1, 1 = profile 2
        # Bit 4 Quick stop active
        quick_stop = bool(logic_msg & (1 << 4))

        # === Curently unused bits ===
        
        # Bit 14 Manual/Automatic
        # pid_automatic = bool(logic_msg & (1 << 14)) # 0 = Manual, 1 = Automatic
        # Bit 12 LOC/REM
        # is_local = bool(logic_msg & (1 << 12)) # 0 = Local, 1 = Remote
        # Bit 11 JOG 
        # jog_active = bool(logic_msg & (1 << 11)) # 0 = No JOG, 1 = JOG active
        # Bit 6 In config mode 
        # in_config_mode = bool(logic_msg & (1 << 6))

    
        error_code = ""
    
        # if fault or alarm present , read and publish info
        if faulted or alarm:
            # fault message
            if faulted:
                # read fault parameter to find fault code
                present_fault = self.read_param("fault_code",type=int)
                # convert fault code to info string
                fault_info = get_fault(present_fault)
                self.get_logger().error(fault_info)
                # print and publish fault info
                print(fault_info)
                error_code = f"F{present_fault}"
                faulted = False # clear alarm flag if fault is present to avoid duplicate messages
            
            if alarm:
                # read alarm parameter to find alarm code
                present_alarm = self.read_param("alarm_code",type=int)
                # convert alarm code to info string
                alarm_info = get_alarm(present_alarm)
                self.get_logger().warn(alarm_info)
                # print and publish alarm info
                print(alarm_info)
                error_code = f"A{present_alarm}"
                alarm = False # clear alarm flag after reporting
        
    
        # Publish treadmill status
        ms = TreadmillStatus()
        ms.speed_mps = speed_mps
        ms.error = error_code
        ms.direction = direction

        self.pub_status.publish(ms)

        

    def on_start(self):

        # set control to vector control mode (just in case)
        self.client.write_register(address=202, value=4, slave=1)

def main(args=None):
    rclpy.init(args=args)

    node = Treadmill()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
