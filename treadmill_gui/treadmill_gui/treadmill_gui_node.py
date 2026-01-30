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
    QGraphicsDropShadowEffect,
)
from PyQt6.QtCore import (
    QTimer,
    Qt,
    QPropertyAnimation,
    QEasingCurve,
    QPoint,
    pyqtProperty,
)
from PyQt6.QtGui import QFont, QColor, QPalette
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class ModernCard(QFrame):
    """Reusable card widget with shadow and modern styling"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet("""
            ModernCard {
                background-color: #1e1e1e;
                border-radius: 12px;
                border: 1px solid #2a2a2a;
            }
        """)

        # Add shadow effect
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        shadow.setColor(QColor(0, 0, 0, 100))
        shadow.setOffset(0, 4)
        self.setGraphicsEffect(shadow)


class AnimatedLabel(QLabel):
    """Label with animated value changes"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._opacity = 1.0

    def flash_update(self):
        """Brief flash animation when value updates"""
        self.setStyleSheet(
            self.styleSheet() + "background-color: rgba(74, 144, 226, 0.2);"
        )
        QTimer.singleShot(
            150,
            lambda: self.setStyleSheet(
                self.styleSheet().replace(
                    "background-color: rgba(74, 144, 226, 0.2);", ""
                )
            ),
        )


class TreadmillGUI(Node):
    def __init__(self):
        # Initialize ROS 2 Node first
        super().__init__("treadmill_gui_node")

        # Create the actual Window object
        self.window = QMainWindow()
        self.window.setWindowTitle("Treadmill Control System")
        self.window.resize(1400, 800)

        # Apply dark theme
        self.apply_modern_theme()

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

    def apply_modern_theme(self):
        """Apply modern dark theme with custom styling"""
        self.window.setStyleSheet("""
            QMainWindow {
                background-color: #121212;
            }
            QLabel {
                color: #e0e0e0;
            }
            QPushButton {
                background-color: #2a2a2a;
                color: #ffffff;
                border: 1px solid #3a3a3a;
                border-radius: 8px;
                padding: 12px 24px;
                font-size: 14px;
                font-weight: 500;
                min-height: 20px;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
                border: 1px solid #4a90e2;
            }
            QPushButton:pressed {
                background-color: #1a1a1a;
            }
            QPushButton:disabled {
                background-color: #1a1a1a;
                color: #666666;
                border: 1px solid #2a2a2a;
            }
            QLineEdit {
                background-color: #2a2a2a;
                color: #ffffff;
                border: 2px solid #3a3a3a;
                border-radius: 8px;
                padding: 10px;
                font-size: 14px;
                selection-background-color: #4a90e2;
            }
            QLineEdit:focus {
                border: 2px solid #4a90e2;
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
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # Header
        header = self.create_header()
        main_layout.addWidget(header)

        # Main content area
        content_layout = QHBoxLayout()
        content_layout.setSpacing(20)

        # Left Panel: Status Display
        status_card = self.create_status_panel()
        content_layout.addWidget(status_card, 2)

        # Middle Panel: Controls
        control_card = self.create_control_panel()
        content_layout.addWidget(control_card, 2)

        # Right Panel: Graph
        graph_card = self.create_graph_panel()
        content_layout.addWidget(graph_card, 3)

        main_layout.addLayout(content_layout)

    def create_header(self):
        """Create modern header with title and status indicator"""
        header_card = ModernCard()
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(24, 16, 24, 16)

        # Title
        title = QLabel("Treadmill Control System")
        title_font = QFont("Segoe UI", 24, QFont.Weight.Bold)
        title.setFont(title_font)
        title.setStyleSheet("color: #ffffff;")

        # Status indicator
        self.status_indicator = QLabel("● OFFLINE")
        self.status_indicator.setStyleSheet("""
            color: #f44336;
            font-size: 14px;
            font-weight: 600;
            padding: 8px 16px;
            background-color: rgba(244, 67, 54, 0.1);
            border-radius: 20px;
        """)

        header_layout.addWidget(title)
        header_layout.addStretch()
        header_layout.addWidget(self.status_indicator)

        return header_card

    def create_status_panel(self):
        """Create status display panel with large readable metrics"""
        card = ModernCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(20)

        # Title
        title = QLabel("METRICS")
        title.setStyleSheet(
            "color: #888888; font-size: 12px; font-weight: 600; letter-spacing: 1px;"
        )
        layout.addWidget(title)

        # Current Speed - Largest display
        current_speed_container = QWidget()
        current_speed_layout = QVBoxLayout(current_speed_container)
        current_speed_layout.setSpacing(4)

        speed_label = QLabel("Current Speed")
        speed_label.setStyleSheet("color: #888888; font-size: 13px;")

        self.lbl_curr_speed = AnimatedLabel("0.00")
        self.lbl_curr_speed.setStyleSheet("""
            color: #4a90e2;
            font-size: 56px;
            font-weight: 700;
            font-family: 'Segoe UI', Arial;
        """)

        speed_unit = QLabel("m/s")
        speed_unit.setStyleSheet("color: #666666; font-size: 18px; font-weight: 500;")

        current_speed_layout.addWidget(speed_label)
        current_speed_layout.addWidget(self.lbl_curr_speed)
        current_speed_layout.addWidget(speed_unit)

        layout.addWidget(current_speed_container)

        # Divider
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setStyleSheet("background-color: #2a2a2a; max-height: 1px;")
        layout.addWidget(divider)

        # Set Speed
        self.lbl_set_speed = self.create_metric_row(
            "Target Speed", "0.00 m/s", "#66bb6a"
        )
        layout.addWidget(self.lbl_set_speed)

        # Distance
        self.lbl_dist = self.create_metric_row("Distance", "0.00 m", "#ffa726")
        layout.addWidget(self.lbl_dist)

        # Time
        self.lbl_time = self.create_metric_row("Time", "0.0 s", "#ab47bc")
        layout.addWidget(self.lbl_time)

        layout.addStretch()
        return card

    def create_metric_row(self, label_text, value_text, color):
        """Create a metric row with label and value"""
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(4)
        layout.setContentsMargins(0, 8, 0, 8)

        label = QLabel(label_text)
        label.setStyleSheet("color: #888888; font-size: 13px;")

        value = QLabel(value_text)
        value.setStyleSheet(f"""
            color: {color};
            font-size: 28px;
            font-weight: 600;
        """)

        layout.addWidget(label)
        layout.addWidget(value)

        # Store reference to value label for updates
        container.value_label = value

        return container

    def create_control_panel(self):
        """Create control panel with buttons and inputs"""
        card = ModernCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        # Title
        title = QLabel("CONTROLS")
        title.setStyleSheet(
            "color: #888888; font-size: 12px; font-weight: 600; letter-spacing: 1px;"
        )
        layout.addWidget(title)

        # Power Button
        self.btn_power = QPushButton("⚡ POWER ON")
        self.btn_power.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2e7d32, stop:1 #43a047);
                color: white;
                font-weight: 600;
                font-size: 15px;
                min-height: 50px;
                border: none;
                border-radius: 10px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #388e3c, stop:1 #4caf50);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #1b5e20, stop:1 #2e7d32);
            }
        """)
        self.btn_power.clicked.connect(self.toggle_power)
        layout.addWidget(self.btn_power)

        # Speed Adjustment Section
        speed_section = QWidget()
        speed_layout = QVBoxLayout(speed_section)
        speed_layout.setSpacing(12)

        speed_title = QLabel("Speed Adjustment")
        speed_title.setStyleSheet("color: #aaaaaa; font-size: 13px; font-weight: 500;")
        speed_layout.addWidget(speed_title)

        # Fine control
        fine_grid = QGridLayout()
        fine_grid.setSpacing(8)

        fine_label = QLabel("Fine (±0.01 m/s)")
        fine_label.setStyleSheet("color: #888888; font-size: 12px;")
        fine_grid.addWidget(fine_label, 0, 0, 1, 2)

        self.btn_up_small = self.create_styled_button(
            "▲", lambda: self.adjust_speed(0.01), "#4a90e2"
        )
        self.btn_dn_small = self.create_styled_button(
            "▼", lambda: self.adjust_speed(-0.01), "#4a90e2"
        )
        fine_grid.addWidget(self.btn_up_small, 1, 0)
        fine_grid.addWidget(self.btn_dn_small, 1, 1)

        # Coarse control
        coarse_label = QLabel("Coarse (±0.1 m/s)")
        coarse_label.setStyleSheet("color: #888888; font-size: 12px;")
        fine_grid.addWidget(coarse_label, 2, 0, 1, 2)

        self.btn_up_large = self.create_styled_button(
            "▲▲", lambda: self.adjust_speed(0.1), "#7b1fa2"
        )
        self.btn_dn_large = self.create_styled_button(
            "▼▼", lambda: self.adjust_speed(-0.1), "#7b1fa2"
        )
        fine_grid.addWidget(self.btn_up_large, 3, 0)
        fine_grid.addWidget(self.btn_dn_large, 3, 1)

        speed_layout.addLayout(fine_grid)

        # Manual input
        input_label = QLabel("Direct Speed Entry")
        input_label.setStyleSheet("color: #888888; font-size: 12px; margin-top: 8px;")
        speed_layout.addWidget(input_label)

        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("Enter speed (m/s)...")
        self.speed_input.returnPressed.connect(self.manual_speed_entry)
        speed_layout.addWidget(self.speed_input)

        layout.addWidget(speed_section)
        layout.addStretch()

        # Emergency Stop
        self.btn_stop = QPushButton("⬛ EMERGENCY STOP")
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #c62828, stop:1 #b71c1c);
                color: white;
                font-weight: 700;
                font-size: 16px;
                min-height: 60px;
                border: 2px solid #d32f2f;
                border-radius: 10px;
                letter-spacing: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #d32f2f, stop:1 #c62828);
                border: 2px solid #f44336;
            }
            QPushButton:pressed {
                background: #b71c1c;
            }
        """)
        self.btn_stop.clicked.connect(self.handle_stop)
        layout.addWidget(self.btn_stop)

        # Reset Button
        self.btn_reset = QPushButton("↻ RESET")
        self.btn_reset.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: #ffffff;
                font-weight: 600;
                font-size: 14px;
                min-height: 40px;
                border: 1px solid #616161;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #616161;
                border: 1px solid #757575;
            }
        """)
        self.btn_reset.clicked.connect(self.handle_reset)
        layout.addWidget(self.btn_reset)

        return card

    def create_styled_button(self, text, callback, color):
        """Create a styled hold button"""
        btn = QPushButton(text)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                font-weight: 600;
                font-size: 18px;
                min-height: 45px;
                border: none;
                border-radius: 8px;
            }}
            QPushButton:hover {{
                background-color: {self.lighten_color(color, 1.2)};
            }}
            QPushButton:pressed {{
                background-color: {self.lighten_color(color, 0.8)};
            }}
        """)
        btn.pressed.connect(lambda: self.start_speed_timer(callback))
        btn.released.connect(self.stop_speed_timer)
        return btn

    def lighten_color(self, hex_color, factor):
        """Lighten or darken a hex color"""
        color = QColor(hex_color)
        h, s, v, a = color.getHsv()
        v = min(255, int(v * factor))
        color.setHsv(h, s, v, a)
        return color.name()

    def create_graph_panel(self):
        """Create graph panel with matplotlib"""
        card = ModernCard()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)

        # Title
        title = QLabel("SPEED PROFILE")
        title.setStyleSheet(
            "color: #888888; font-size: 12px; font-weight: 600; letter-spacing: 1px;"
        )
        layout.addWidget(title)

        # Graph
        self.fig = Figure(figsize=(6, 4), dpi=100, facecolor="#1e1e1e")
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)

        # Style the plot
        self.ax.set_facecolor("#1e1e1e")
        self.ax.tick_params(colors="#888888", labelsize=9)
        self.ax.spines["bottom"].set_color("#3a3a3a")
        self.ax.spines["left"].set_color("#3a3a3a")
        self.ax.spines["top"].set_color("#3a3a3a")
        self.ax.spines["right"].set_color("#3a3a3a")
        self.ax.grid(True, alpha=0.15, color="#888888", linestyle="--", linewidth=0.5)
        self.ax.set_xlabel("Time (s)", color="#888888", fontsize=10)
        self.ax.set_ylabel("Speed (m/s)", color="#888888", fontsize=10)

        (self.line,) = self.ax.plot(
            [], [], color="#4a90e2", linewidth=2.5, antialiased=True
        )

        # Add glow effect to line
        self.ax.plot([], [], color="#4a90e2", linewidth=5, alpha=0.3, antialiased=True)

        layout.addWidget(self.canvas)
        return card

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
        prev_speed = self.current_speed_mps
        self.current_speed_mps = abs(msg.speed_mps)

        # Flash animation on significant change
        if abs(self.current_speed_mps - prev_speed) > 0.05:
            self.lbl_curr_speed.flash_update()

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

            # Style the message box
            m.setStyleSheet("""
                QMessageBox {
                    background-color: #1e1e1e;
                    color: #e0e0e0;
                }
                QMessageBox QLabel {
                    color: #e0e0e0;
                    font-size: 13px;
                }
                QPushButton {
                    background-color: #2a2a2a;
                    color: #ffffff;
                    border: 1px solid #3a3a3a;
                    border-radius: 6px;
                    padding: 8px 16px;
                    min-width: 70px;
                }
                QPushButton:hover {
                    background-color: #3a3a3a;
                }
            """)

            m.setText(
                f"<b style='color: #f44336; font-size: 15px;'>Description:</b><br>{details['desc']}"
            )
            m.setInformativeText(f"<b>Possible Causes:</b><br>{details['causes']}")
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
            self.canvas.draw_idle()  # More efficient than draw()

        # Update displays
        self.lbl_curr_speed.setText(f"{self.current_speed_mps:.2f}")
        self.lbl_set_speed.value_label.setText(f"{self.set_speed_mps:.2f} m/s")
        self.lbl_dist.value_label.setText(f"{self.distance_traveled:.2f} m")
        self.lbl_time.value_label.setText(f"{self.time_running:.1f} s")

        # Update status indicator
        if self.is_powered_on:
            if self.is_stopped or self.set_speed_mps == 0:
                self.status_indicator.setText("● STANDBY")
                self.status_indicator.setStyleSheet("""
                    color: #ffa726;
                    font-size: 14px;
                    font-weight: 600;
                    padding: 8px 16px;
                    background-color: rgba(255, 167, 38, 0.1);
                    border-radius: 20px;
                """)
            else:
                self.status_indicator.setText("● RUNNING")
                self.status_indicator.setStyleSheet("""
                    color: #66bb6a;
                    font-size: 14px;
                    font-weight: 600;
                    padding: 8px 16px;
                    background-color: rgba(102, 187, 106, 0.1);
                    border-radius: 20px;
                """)
        else:
            self.status_indicator.setText("● OFFLINE")
            self.status_indicator.setStyleSheet("""
                color: #f44336;
                font-size: 14px;
                font-weight: 600;
                padding: 8px 16px;
                background-color: rgba(244, 67, 54, 0.1);
                border-radius: 20px;
            """)

    def toggle_power(self):
        self.is_powered_on = not self.is_powered_on
        cmd = "power_on" if self.is_powered_on else "power_off"
        self.pub_cmd.publish(String(data=cmd))

        if self.is_powered_on:
            self.btn_power.setText("⚡ POWER OFF")
            self.btn_power.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #d32f2f, stop:1 #f44336);
                    color: white;
                    font-weight: 600;
                    font-size: 15px;
                    min-height: 50px;
                    border: none;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #e53935, stop:1 #ef5350);
                }
                QPushButton:pressed {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #c62828, stop:1 #d32f2f);
                }
            """)
        else:
            self.btn_power.setText("⚡ POWER ON")
            self.btn_power.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #2e7d32, stop:1 #43a047);
                    color: white;
                    font-weight: 600;
                    font-size: 15px;
                    min-height: 50px;
                    border: none;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #388e3c, stop:1 #4caf50);
                }
                QPushButton:pressed {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #1b5e20, stop:1 #2e7d32);
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

        # Clear and restyle graph
        self.ax.clear()
        self.ax.set_facecolor("#1e1e1e")
        self.ax.tick_params(colors="#888888", labelsize=9)
        self.ax.spines["bottom"].set_color("#3a3a3a")
        self.ax.spines["left"].set_color("#3a3a3a")
        self.ax.spines["top"].set_color("#3a3a3a")
        self.ax.spines["right"].set_color("#3a3a3a")
        self.ax.grid(True, alpha=0.15, color="#888888", linestyle="--", linewidth=0.5)
        self.ax.set_xlabel("Time (s)", color="#888888", fontsize=10)
        self.ax.set_ylabel("Speed (m/s)", color="#888888", fontsize=10)
        (self.line,) = self.ax.plot(
            [], [], color="#4a90e2", linewidth=2.5, antialiased=True
        )
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
            if val < 0:
                return
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
