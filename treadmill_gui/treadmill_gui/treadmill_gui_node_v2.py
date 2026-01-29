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
    QFrame,
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QPalette, QColor
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class TreadmillGUI(Node):
    def __init__(self):
        # Initialize ROS 2 Node first
        super().__init__("treadmill_gui_node")

        # Create the actual Window object
        self.window = QMainWindow()
        self.window.setWindowTitle("Treadmill Control System")
        self.window.resize(1200, 750)

        # Set modern dark theme
        self.setup_theme()

        # --- Error Databases ---
        self.faults = {
            6: {
                "desc": "Imbalance or Input Phase Loss",
                "causes": "Mains voltage imbalance > 5%.",
            },
            21: {
                "desc": "DC Link Undervoltage",
                "causes": "Input voltage too low; Phase loss.",
            },
            22: {
                "desc": "DC Link Overvoltage",
                "causes": "Inertia too high; Decel time too short.",
            },
            30: {
                "desc": "Power Module U Fault",
                "causes": "Short-circuit between phases.",
            },
            34: {
                "desc": "Power Module V Fault",
                "causes": "Short-circuit between phases.",
            },
            38: {
                "desc": "Power Module W Fault",
                "causes": "Short-circuit between phases.",
            },
            42: {
                "desc": "DB IGBT Fault",
                "causes": "Short-circuit in braking resistor.",
            },
            48: {
                "desc": "IGBT Overload Fault",
                "causes": "High current; check switching freq.",
            },
            51: {
                "desc": "IGBT Overtemperature",
                "causes": "NTC sensors detected high heat.",
            },
            71: {
                "desc": "Output Overcurrent",
                "causes": "Excessive load; Accel time too short.",
            },
            72: {"desc": "Motor Overload", "causes": "Motor shaft load is excessive."},
            74: {
                "desc": "Ground Fault",
                "causes": "Short circuit to ground in motor/cables.",
            },
            78: {
                "desc": "Motor Overtemperature",
                "causes": "Excessive duty cycle; Ambient temp high.",
            },
            156: {"desc": "Undertemperature", "causes": "Air temperature <= -30 °C."},
            185: {
                "desc": "Pre-charge Contactor Fault",
                "causes": "Phase loss; Contactor defect.",
            },
        }

        self.alarms = {
            46: {"desc": "High Load on Motor", "causes": "Motor shaft load excessive."},
            47: {
                "desc": "IGBT Overload Alarm",
                "causes": "High current at inverter output.",
            },
            50: {
                "desc": "IGBT High Temperature",
                "causes": "Ambient temp > 45°C; Fan blocked.",
            },
            88: {"desc": "Communication Lost", "causes": "Loose keypad cable; Noise."},
            90: {
                "desc": "External Alarm",
                "causes": "External safety input triggered.",
            },
            110: {"desc": "High Motor Temperature", "causes": "Excessive shaft load."},
            128: {
                "desc": "Timeout for Serial Communication",
                "causes": "No valid messages; Check wiring.",
            },
            152: {
                "desc": "Internal Air High Temperature",
                "causes": "Internal fan defective.",
            },
            177: {"desc": "Fan Replacement", "causes": "Heatsink fan > 50,000 hours."},
            702: {
                "desc": "Inverter Disabled",
                "causes": "General Enable command not active.",
            },
        }

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

        # Setup UI and ROS
        self.init_ui()
        self.init_ros()

        # Update GUI timer (100ms)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui_loop)
        self.timer.start(100)

    def setup_theme(self):
        """Apply modern dark theme to the application"""
        self.window.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e2e;
            }
            QWidget {
                background-color: #1e1e2e;
                color: #cdd6f4;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 13px;
            }
            QPushButton {
                background-color: #313244;
                color: #cdd6f4;
                border: 2px solid #45475a;
                border-radius: 8px;
                padding: 12px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45475a;
                border: 2px solid #585b70;
            }
            QPushButton:pressed {
                background-color: #585b70;
            }
            QLineEdit {
                background-color: #313244;
                color: #cdd6f4;
                border: 2px solid #45475a;
                border-radius: 6px;
                padding: 8px;
                font-size: 13px;
            }
            QLineEdit:focus {
                border: 2px solid #89b4fa;
            }
            QLabel {
                color: #cdd6f4;
            }
            QFrame {
                background-color: #181825;
                border-radius: 10px;
                padding: 15px;
            }
        """)

    def init_ros(self):
        self.pub_speed = self.create_publisher(Float32, "treadmill/cmd_speed_mps", 10)
        self.pub_cmd = self.create_publisher(String, "treadmill/special_cmd", 10)
        self.sub_status = self.create_subscription(
            TreadmillStatus, "treadmill/status", self.status_callback, 10
        )

    def init_ui(self):
        central_widget = QWidget()
        self.window.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # --- Left Panel: Controls ---
        ctrl_frame = QFrame()
        ctrl_frame.setMaximumWidth(300)
        ctrl_layout = QVBoxLayout(ctrl_frame)
        ctrl_layout.setSpacing(15)

        # Power button with dynamic styling
        self.btn_power = QPushButton("⚡ POWER ON")
        self.btn_power.setMinimumHeight(50)
        self.btn_power.setStyleSheet("""
            QPushButton {
                background-color: #a6e3a1;
                color: #1e1e2e;
                border: none;
                border-radius: 10px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #b8f0b6;
            }
        """)
        self.btn_power.clicked.connect(self.toggle_power)

        # Speed control section
        speed_label = QLabel("⚙️ SPEED CONTROL")
        speed_label.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #89b4fa; margin-top: 10px;"
        )

        speed_grid = QGridLayout()
        speed_grid.setSpacing(8)

        fine_label = QLabel("Fine Adjust (±0.01 m/s)")
        fine_label.setStyleSheet("font-size: 11px; color: #a6adc8;")
        self.btn_up_small = self.create_hold_button(
            "▲", lambda: self.adjust_speed(0.01)
        )
        self.btn_dn_small = self.create_hold_button(
            "▼", lambda: self.adjust_speed(-0.01)
        )

        coarse_label = QLabel("Coarse Adjust (±0.1 m/s)")
        coarse_label.setStyleSheet("font-size: 11px; color: #a6adc8;")
        self.btn_up_large = self.create_hold_button(
            "▲▲", lambda: self.adjust_speed(0.1)
        )
        self.btn_dn_large = self.create_hold_button(
            "▼▼", lambda: self.adjust_speed(-0.1)
        )

        # Style speed buttons
        speed_btn_style = """
            QPushButton {
                background-color: #89b4fa;
                color: #1e1e2e;
                border: none;
                border-radius: 6px;
                font-size: 18px;
                font-weight: bold;
                min-height: 45px;
            }
            QPushButton:hover {
                background-color: #a4c8fc;
            }
            QPushButton:pressed {
                background-color: #6c9edb;
            }
        """
        for btn in [
            self.btn_up_small,
            self.btn_dn_small,
            self.btn_up_large,
            self.btn_dn_large,
        ]:
            btn.setStyleSheet(speed_btn_style)

        speed_grid.addWidget(fine_label, 0, 0, 1, 2, Qt.AlignmentFlag.AlignCenter)
        speed_grid.addWidget(self.btn_up_small, 1, 0)
        speed_grid.addWidget(self.btn_dn_small, 1, 1)
        speed_grid.addWidget(coarse_label, 2, 0, 1, 2, Qt.AlignmentFlag.AlignCenter)
        speed_grid.addWidget(self.btn_up_large, 3, 0)
        speed_grid.addWidget(self.btn_dn_large, 3, 1)

        # Manual speed entry
        manual_label = QLabel("Manual Entry:")
        manual_label.setStyleSheet("font-size: 12px; color: #a6adc8; margin-top: 10px;")
        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("Enter speed (m/s)...")
        self.speed_input.returnPressed.connect(self.manual_speed_entry)
        self.speed_input.setMinimumHeight(40)

        # Emergency stop button
        self.btn_stop = QPushButton("⬛ EMERGENCY STOP")
        self.btn_stop.setMinimumHeight(70)
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #f38ba8;
                color: #1e1e2e;
                border: none;
                border-radius: 10px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #f5a3b5;
            }
            QPushButton:pressed {
                background-color: #d86b87;
            }
        """)
        self.btn_stop.clicked.connect(self.handle_stop)

        # Reset button
        self.btn_reset = QPushButton("🔄 RESET")
        self.btn_reset.setMinimumHeight(45)
        self.btn_reset.setStyleSheet("""
            QPushButton {
                background-color: #fab387;
                color: #1e1e2e;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #fcc69f;
            }
        """)
        self.btn_reset.clicked.connect(self.handle_reset)

        ctrl_layout.addWidget(self.btn_power)
        ctrl_layout.addWidget(speed_label)
        ctrl_layout.addLayout(speed_grid)
        ctrl_layout.addWidget(manual_label)
        ctrl_layout.addWidget(self.speed_input)
        ctrl_layout.addStretch()
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.btn_reset)

        # --- Middle Panel: Status Display ---
        display_frame = QFrame()
        display_frame.setMaximumWidth(350)
        display_layout = QVBoxLayout(display_frame)
        display_layout.setSpacing(20)

        # Title
        title_label = QLabel("📊 SYSTEM STATUS")
        title_label.setStyleSheet(
            "font-size: 16px; font-weight: bold; color: #89b4fa; margin-bottom: 10px;"
        )
        display_layout.addWidget(title_label)

        # Status displays with icons
        self.lbl_curr_speed = self.create_status_label("🏃 Current Speed", "0.00 m/s")
        self.lbl_set_speed = self.create_status_label("🎯 Target Speed", "0.00 m/s")
        self.lbl_dist = self.create_status_label("📏 Distance", "0.00 m")
        self.lbl_time = self.create_status_label("⏱️ Time Running", "0.0 s")

        for lbl_pair in [
            self.lbl_curr_speed,
            self.lbl_set_speed,
            self.lbl_dist,
            self.lbl_time,
        ]:
            container = QFrame()
            container.setStyleSheet("""
                QFrame {
                    background-color: #313244;
                    border-radius: 8px;
                    padding: 12px;
                    margin: 3px;
                }
            """)
            container_layout = QVBoxLayout(container)
            container_layout.setContentsMargins(10, 10, 10, 10)
            container_layout.addWidget(lbl_pair[0])
            container_layout.addWidget(lbl_pair[1])
            display_layout.addWidget(container)

        display_layout.addStretch()

        # --- Right Panel: Graph ---
        graph_frame = QFrame()
        graph_layout = QVBoxLayout(graph_frame)

        graph_title = QLabel("📈 SPEED MONITOR")
        graph_title.setStyleSheet(
            "font-size: 16px; font-weight: bold; color: #89b4fa; margin-bottom: 10px;"
        )
        graph_layout.addWidget(graph_title)

        self.fig = Figure(figsize=(6, 5), dpi=100, facecolor="#181825")
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, facecolor="#1e1e2e")
        self.ax.set_title(
            "Speed vs Time", color="#cdd6f4", fontsize=14, fontweight="bold"
        )
        self.ax.set_xlabel("Time (s)", color="#a6adc8", fontsize=11)
        self.ax.set_ylabel("Speed (m/s)", color="#a6adc8", fontsize=11)
        self.ax.tick_params(colors="#a6adc8", labelsize=9)
        self.ax.spines["bottom"].set_color("#45475a")
        self.ax.spines["top"].set_color("#45475a")
        self.ax.spines["left"].set_color("#45475a")
        self.ax.spines["right"].set_color("#45475a")
        self.ax.grid(True, alpha=0.2, color="#45475a", linestyle="--")
        (self.line,) = self.ax.plot([], [], color="#89b4fa", linewidth=2)

        graph_layout.addWidget(self.canvas)

        # Add all panels to main layout
        main_layout.addWidget(ctrl_frame, 1)
        main_layout.addWidget(display_frame, 1)
        main_layout.addWidget(graph_frame, 2)

    def create_status_label(self, title, initial_value):
        """Create a styled status label pair"""
        title_label = QLabel(title)
        title_label.setStyleSheet(
            "font-size: 12px; color: #a6adc8; font-weight: normal;"
        )

        value_label = QLabel(initial_value)
        value_label.setStyleSheet(
            "font-size: 24px; color: #cdd6f4; font-weight: bold; font-family: 'Courier New', monospace;"
        )

        return (title_label, value_label)

    def create_hold_button(self, text, func):
        btn = QPushButton(text)
        btn.pressed.connect(lambda: self.start_speed_timer(func))
        btn.released.connect(self.stop_speed_timer)
        return btn

    def start_speed_timer(self, func):
        func()
        self.active_timer = QTimer()
        self.active_timer.setInterval(200)
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
            m = QMessageBox(self.window)
            m.setIcon(
                QMessageBox.Icon.Critical if prefix == "F" else QMessageBox.Icon.Warning
            )
            m.setWindowTitle(f"Inverter {prefix}{code:03d}")
            m.setText(f"<b>Description:</b> {details['desc']}")
            m.setInformativeText(f"<b>Possible Causes:</b><br>{details['causes']}")

            # Style the message box
            m.setStyleSheet("""
                QMessageBox {
                    background-color: #1e1e2e;
                    color: #cdd6f4;
                }
                QMessageBox QLabel {
                    color: #cdd6f4;
                }
                QPushButton {
                    background-color: #313244;
                    color: #cdd6f4;
                    border: 2px solid #45475a;
                    border-radius: 6px;
                    padding: 8px 16px;
                    min-width: 80px;
                }
                QPushButton:hover {
                    background-color: #45475a;
                }
            """)
            m.exec()

    def update_gui_loop(self):
        rclpy.spin_once(self, timeout_sec=0)

        if self.is_powered_on and not self.is_stopped and self.set_speed_mps != 0:
            self.time_running += 0.1
            self.distance_traveled += self.current_speed_mps * 0.1
            self.time_history.append(self.time_running)
            self.speed_history.append(self.current_speed_mps)

            if len(self.time_history) > 300:
                self.time_history.pop(0)
                self.speed_history.pop(0)

            self.line.set_data(self.time_history, self.speed_history)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw()

        # Update status displays
        self.lbl_curr_speed[1].setText(f"{self.current_speed_mps:.2f} m/s")
        self.lbl_set_speed[1].setText(f"{self.set_speed_mps:.2f} m/s")
        self.lbl_dist[1].setText(f"{self.distance_traveled:.2f} m")
        self.lbl_time[1].setText(f"{self.time_running:.1f} s")

    def toggle_power(self):
        self.is_powered_on = not self.is_powered_on
        cmd = "power_on" if self.is_powered_on else "power_off"
        self.pub_cmd.publish(String(data=cmd))

        if self.is_powered_on:
            self.btn_power.setText("⚡ POWER OFF")
            self.btn_power.setStyleSheet("""
                QPushButton {
                    background-color: #f38ba8;
                    color: #1e1e2e;
                    border: none;
                    border-radius: 10px;
                    font-size: 16px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #f5a3b5;
                }
            """)
        else:
            self.btn_power.setText("⚡ POWER ON")
            self.btn_power.setStyleSheet("""
                QPushButton {
                    background-color: #a6e3a1;
                    color: #1e1e2e;
                    border: none;
                    border-radius: 10px;
                    font-size: 16px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #b8f0b6;
                }
            """)

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
        self.ax.set_facecolor("#1e1e2e")
        self.ax.set_title(
            "Speed vs Time", color="#cdd6f4", fontsize=14, fontweight="bold"
        )
        self.ax.set_xlabel("Time (s)", color="#a6adc8", fontsize=11)
        self.ax.set_ylabel("Speed (m/s)", color="#a6adc8", fontsize=11)
        self.ax.tick_params(colors="#a6adc8", labelsize=9)
        self.ax.grid(True, alpha=0.2, color="#45475a", linestyle="--")
        (self.line,) = self.ax.plot([], [], color="#89b4fa", linewidth=2)
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
        except ValueError:
            pass


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui_node = TreadmillGUI()
    gui_node.window.show()

    try:
        sys.exit(app.exec())
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
