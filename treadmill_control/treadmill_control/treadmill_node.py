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

# placeholder
RPM_TO_MPS = 0.22 # Conversion factor from RPM to meters per second

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
    """
    faults = {
        6: {"desc": "Imbalance or Input Phase Loss", "causes": "Mains voltage imbalance too high; phase missing."},
        21: {"desc": "DC Link Undervoltage", "causes": "Input voltage too low; Phase loss."},
        22: {"desc": "DC Link Overvoltage", "causes": "Inertia too high; Decel time too short."},
        30: {"desc": "Power Module U Fault", "causes": "Short-circuit U-V or U-W."},
        34: {"desc": "Power Module V Fault", "causes": "Short-circuit V-U or V-W."},
        38: {"desc": "Power Module W Fault", "causes": "Short-circuit W-U or W-V."},
        42: {"desc": "DB IGBT Fault", "causes": "Braking resistor short."},
        48: {"desc": "IGBT Overload Fault", "causes": "High current at output."},
        51: {"desc": "IGBT Overtemperature", "causes": "High temp on IGBTs."},
        71: {"desc": "Output Overcurrent", "causes": "Excessive load inertia; Accel time too short."},
        72: {"desc": "Motor Overload", "causes": "Motor shaft load is excessive."},
        74: {"desc": "Ground Fault", "causes": "Short circuit to ground."},
        78: {"desc": "Motor Overtemperature", "causes": "Excessive load/duty cycle."},
        156: {"desc": "Undertemperature", "causes": "Air temp <= -30 °C."},
        185: {"desc": "Pre-charge Contactor Fault", "causes": "Open command fuse; Contactor defect."}
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
    """
    alarms = {
        46: {"desc": "High Load on Motor", "causes": "High load."},
        47: {"desc": "IGBT Overload Alarm", "causes": "High current."},
        50: {"desc": "IGBT High Temperature",
