import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from treadmill_interfaces.msg import TreadmillStatus

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QLineEdit,
    QMessageBox,
    QGridLayout,
)
from PyQt6.QtCore import QTimer, Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class TreadmillGUI(QMainWindow, Node):
    def __init__(self):
        # Initialize ROS 2 Node
        Node.__init__(self, "treadmill_gui_node")

        # --- Error Databases (from treadmill_node.py) ---
        self.faults = {
            6: {
                "desc": "Imbalance or Input Phase Loss",
                "causes": "Mains voltage imbalance too high; phase missing at input; input voltage imbalance > 5%.",
            },
            21: {
                "desc": "DC Link Undervoltage",
                "causes": "Input voltage too low; Phase loss; Pre-charge circuit failure; P0296 set too high.",
            },
            22: {
                "desc": "DC Link Overvoltage",
                "causes": "Inertia of driven-load too high; Deceleration time too short; P0151/P0153/P0185 set too high.",
            },
            30: {
                "desc": "Power Module U Fault",
                "causes": "IGBT desaturation; Short-circuit between phases U-V or U-W.",
            },
            34: {
                "desc": "Power Module V Fault",
                "causes": "IGBT desaturation; Short-circuit between phases V-U or V-W.",
            },
            38: {
                "desc": "Power Module W Fault",
                "causes": "IGBT desaturation; Short-circuit between phases W-U or W-V.",
            },
            42: {
                "desc": "DB IGBT Fault",
                "causes": "Desaturation of Dynamic Braking IGBT; Short-circuit in braking resistor cables.",
            },
            48: {
                "desc": "IGBT Overload Fault",
                "causes": "High current at inverter output; check switching frequency vs. load.",
            },
            51: {
                "desc": "IGBT Overtemperature",
                "causes": "High temperature detected by NTC sensors on IGBTs.",
            },
            71: {
                "desc": "Output Overcurrent",
                "causes": "Excessive load inertia; Acceleration time too short; P0135/P0169/P0170 too high.",
            },
            72: {
                "desc": "Motor Overload",
                "causes": "P0156/P0157/P0158 settings too low; Motor shaft load is excessive.",
            },
            74: {
                "desc": "Ground Fault",
                "causes": "Short circuit to ground in motor/cables; Motor cable capacitance too large.",
            },
            78: {
                "desc": "Motor Overtemperature",
                "causes": "Excessive motor load/duty cycle; Ambient temp too high; PTC wiring issues.",
            },
            156: {
                "desc": "Undertemperature",
                "causes": "Surrounding air temperature <= -30 °C (-22 °F).",
            },
            185: {
                "desc": "Pre-charge Contactor Fault",
                "causes": "Open command fuse; Phase loss in L1/R or L2/S; Contactor defect.",
            },
        }

        self.alarms = {
            46: {
                "desc": "High Load on Motor",
                "causes": "Settings of P0156/P0157/P0158 too low; Motor shaft load excessive.",
            },
            47: {
                "desc": "IGBT Overload Alarm",
                "causes": "High current at inverter output; Check switching frequency table.",
            },
            50: {
                "desc": "IGBT High Temperature",
                "causes": "High ambient temp (> 45°C); Fan blocked/defective; Dirty heatsink.",
            },
            88: {
                "desc": "Communication Lost",
                "causes": "Loose keypad cable; Electrical noise in installation.",
            },
            90: {
                "desc": "External Alarm",
                "causes": "Digital input set to 'No External Alarm' is open/unwired.",
            },
            110: {
                "desc": "High Motor Temperature",
                "causes": "Excessive shaft load; High ambient temp; PTC sensor wiring issues.",
            },
            128: {
                "desc": "Timeout for Serial Communication",
                "causes": "No valid messages received within P0314 interval; Check wiring/grounding.",
            },
            152: {
                "desc": "Internal Air High Temperature",
                "causes": "Ambient temp > 45°C; Internal fan defective.",
            },
            177: {
                "desc": "Fan Replacement",
                "causes": "Heatsink fan has exceeded 50,000 operating hours.",
            },
            702: {
                "desc": "Inverter Disabled",
                "causes": "General Enable command not active while SoftPLC is in Run mode.",
            },
        }

        # Initialize PyQt Window
        QMainWindow.__init__(self)
        self.setWindowTitle("Treadmill Control System")
        self.resize(1100, 700)

        # State Variables
        self.current_speed_mps = 0.0
        self.set_speed_mps = 0.0
        self.distance_traveled = 0.0
        self.time_running = 0.0
        self.is_powered_on = False
        self.is_stopped = True
        self.last_error = ""
        self.speed_history = []
        self.time_history = []

        self.active_timer = None

        self.init_ui()
        self.init_ros()

        # Update GUI timer (100ms)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui_loop)
        self.timer.start(100)

    def init_ros(self):
        self.pub_speed = self.create_publisher(Float32, "treadmill/cmd_speed_mps", 10)
        self.pub_cmd = self.create_publisher(String, "treadmill/special_cmd", 10)
        self.sub_status = self.create_subscription(
            TreadmillStatus, "treadmill/status", self.status_callback, 10
        )

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- Left Panel: Controls ---
        ctrl_layout = QVBoxLayout()

        self.btn_power = QPushButton("Power ON")
        self.btn_power.clicked.connect(self.toggle_power)

        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet(
            "background-color: #d32f2f; color: white; font-weight: bold; height: 60px;"
        )
        self.btn_stop.clicked.connect(self.handle_stop)

        # Speed Controls Grid
        speed_grid = QGridLayout()
        self.btn_up_small = self.create_hold_button(
            "▲ 0.01", lambda: self.adjust_speed(0.01)
        )
        self.btn_dn_small = self.create_hold_button(
            "▼ 0.01", lambda: self.adjust_speed(-0.01)
        )
        self.btn_up_large = self.create_hold_button(
            "▲ 0.1", lambda: self.adjust_speed(0.1)
        )
        self.btn_dn_large = self.create_hold_button(
            "▼ 0.1", lambda: self.adjust_speed(-0.1)
        )

        speed_grid.addWidget(
            QLabel("Fine (0.01)"), 0, 0, 1, 2, Qt.AlignmentFlag.AlignCenter
        )
        speed_grid.addWidget(self.btn_up_small, 1, 0)
        speed_grid.addWidget(self.btn_dn_small, 1, 1)
        speed_grid.addWidget(
            QLabel("Coarse (0.1)"), 2, 0, 1, 2, Qt.AlignmentFlag.AlignCenter
        )
        speed_grid.addWidget(self.btn_up_large, 3, 0)
        speed_grid.addWidget(self.btn_dn_large, 3, 1)

        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("Enter mps...")
        self.speed_input.returnPressed.connect(self.manual_speed_entry)

        self.btn_reset = QPushButton("RESET")
        self.btn_reset.clicked.connect(self.handle_reset)

        ctrl_layout.addWidget(self.btn_power)
        ctrl_layout.addLayout(speed_grid)
        ctrl_layout.addWidget(QLabel("Manual Speed Entry:"))
        ctrl_layout.addWidget(self.speed_input)
        ctrl_layout.addStretch()
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.btn_reset)

        # --- Middle Panel: Displays ---
        display_layout = QVBoxLayout()
        self.lbl_curr_speed = QLabel("Current: 0.00 mps")
        self.lbl_set_speed = QLabel("Set: 0.00 mps")
        self.lbl_dist = QLabel("Distance: 0.00 m")
        self.lbl_time = QLabel("Time: 0.0s")

        for lbl in [
            self.lbl_curr_speed,
            self.lbl_set_speed,
            self.lbl_dist,
            self.lbl_time,
        ]:
            lbl.setStyleSheet(
                "font-family: monospace; font-size: 20px; font-weight: bold;"
            )
            display_layout.addWidget(lbl)

        # --- Right Panel: Graph ---
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Speed vs Time")
        self.ax.set_ylabel("mps")
        (self.line,) = self.ax.plot([], [], "b-")

        main_layout.addLayout(ctrl_layout, 1)
        main_layout.addLayout(display_layout, 1)
        main_layout.addWidget(self.canvas, 3)

    def create_hold_button(self, text, func):
        btn = QPushButton(text)
        btn.pressed.connect(lambda: self.start_speed_timer(func))
        btn.released.connect(self.stop_speed_timer)
        return btn

    def start_speed_timer(self, func):
        func()
        self.active_timer = QTimer()
        self.active_timer.setInterval(200)  # 5 triggers per second
        self.active_timer.timeout.connect(func)
        self.active_timer.start()

    def stop_speed_timer(self):
        if self.active_timer:
            self.active_timer.stop()
            self.active_timer = None

    def status_callback(self, msg):
        self.current_speed_mps = abs(msg.speed_mps)
        if msg.error and msg.error != self.last_error:
            self.last_error = msg.error
            self.handle_error_popup(msg.error)

    def handle_error_popup(self, error_str):
        prefix = error_str[0]
        try:
            code = int(error_str[1:])
        except:
            return

        details = self.faults.get(code) if prefix == "F" else self.alarms.get(code)

        if details:
            m = QMessageBox(self)
            m.setIcon(
                QMessageBox.Icon.Critical if prefix == "F" else QMessageBox.Icon.Warning
            )
            m.setWindowTitle(f"Inverter {prefix}{code:03d}")
            m.setText(f"<b>Description:</b> {details['desc']}")
            m.setInformativeText(f"<b>Possible Causes:</b><br>{details['causes']}")
            m.exec()

    def update_gui_loop(self):
        rclpy.spin_once(self, timeout_sec=0)

        if self.is_powered_on and not self.is_stopped and self.set_speed_mps != 0:
            self.time_running += 0.1
            self.distance_traveled += self.current_speed_mps * 0.1
            self.time_history.append(self.time_running)
            self.speed_history.append(self.current_speed_mps)

            if len(self.time_history) > 300:  # 30s window
                self.time_history.pop(0)
                self.speed_history.pop(0)

            self.line.set_data(self.time_history, self.speed_history)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw()

        self.lbl_curr_speed.setText(f"Current: {self.current_speed_mps:.2f} mps")
        self.lbl_set_speed.setText(f"Set: {self.set_speed_mps:.2f} mps")
        self.lbl_dist.setText(f"Distance: {self.distance_traveled:.2f} m")
        self.lbl_time.setText(f"Time: {self.time_running:.1f}s")

    def toggle_power(self):
        self.is_powered_on = not self.is_powered_on
        cmd = "power_on" if self.is_powered_on else "power_off"
        self.pub_cmd.publish(String(data=cmd))
        self.btn_power.setText("Power OFF" if self.is_powered_on else "Power ON")

    def handle_stop(self):
        self.is_stopped = True
        self.set_speed_mps = 0.0
        self.pub_cmd.publish(String(data="stop"))
        self.pub_speed.publish(Float32(data=0.0))

    def handle_reset(self):
        self.pub_cmd.publish(String(data="power_off"))
        QTimer.singleShot(500, lambda: self.pub_cmd.publish(String(data="power_on")))
        self.pub_cmd.publish(String(data="reset_fault"))
        self.time_running = 0.0
        self.distance_traveled = 0.0
        self.set_speed_mps = 0.0
        self.last_error = ""
        self.speed_history.clear()
        self.time_history.clear()
        self.ax.clear()
        self.ax.set_title("Speed vs Time")
        (self.line,) = self.ax.plot([], [], "b-")
        self.canvas.draw()

    def adjust_speed(self, delta):
        if not self.is_powered_on:
            return
        self.is_stopped = False
        self.set_speed_mps = max(0.0, self.set_speed_mps + delta)
        self.pub_speed.publish(Float32(data=self.set_speed_mps))
        self.pub_cmd.publish(String(data="go"))

    def manual_speed_entry(self):
        try:
            val = float(self.speed_input.text())
            self.set_speed_mps = val
            self.is_stopped = False
            self.pub_speed.publish(Float32(data=val))
            self.pub_cmd.publish(String(data="go"))
            self.speed_input.clear()
        except:
            pass


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = TreadmillGUI()
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
