#!/usr/bin/env python3
"""
Treadmill Control GUI
Modern PyQt6 interface for ROS2 treadmill control node
"""

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
    QGridLayout,
    QPushButton,
    QLabel,
    QSlider,
    QLineEdit,
    QFrame,
    QMessageBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
from datetime import datetime, timedelta
import threading


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


class ROSSignals(QObject):
    """Signal bridge between ROS callbacks and Qt GUI"""

    status_updated = pyqtSignal(object)


class TreadmillGUINode(Node):
    """ROS2 node for treadmill GUI communication"""

    def __init__(self, signals):
        super().__init__("treadmill_gui")
        self.signals = signals

        # Publishers
        self.pub_cmd_speed = self.create_publisher(
            Float32, "treadmill/cmd_speed_mps", 10
        )
        self.pub_special_cmd = self.create_publisher(
            String, "treadmill/special_cmd", 10
        )

        # Subscriber
        self.create_subscription(
            TreadmillStatus, "treadmill/status", self.status_callback, 10
        )

        self.get_logger().info("Treadmill GUI Node initialized")

    def status_callback(self, msg):
        """Forward status updates to GUI via signals"""
        self.signals.status_updated.emit(msg)

    def send_speed(self, speed_mps):
        """Send speed command"""
        msg = Float32()
        msg.data = float(speed_mps)
        self.pub_cmd_speed.publish(msg)

    def send_special_command(self, command):
        """Send special command (start, stop, on, off, etc.)"""
        msg = String()
        msg.data = command
        self.pub_special_cmd.publish(msg)


class TreadmillGUI(QMainWindow):
    """Main GUI window for treadmill control"""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        # State variables
        self.current_speed_mps = 0.0
        self.set_speed_mps = 0.0
        self.distance_m = 0.0
        self.running_time = timedelta(0)
        self.is_powered = False
        self.is_running = False
        self.last_update_time = None

        # Graph data
        self.time_data = []
        self.speed_data = []
        self.max_graph_points = 300  # 30 seconds at 10Hz

        # UI update timer
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_time_and_distance)
        self.ui_timer.setInterval(100)  # 10Hz

        # Speed button hold timers
        self.speed_hold_timer = QTimer()
        self.speed_hold_timer.timeout.connect(self.handle_speed_hold)
        self.speed_hold_active = None

        self.init_ui()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Treadmill Control System")
        self.setGeometry(100, 100, 1200, 800)

        # Apply modern industrial style
        self.apply_stylesheet()

        # Central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # Left panel - Controls
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)

        # Right panel - Display and Graph
        right_panel = self.create_display_panel()
        main_layout.addWidget(right_panel, 2)

        self.show()

    def apply_stylesheet(self):
        """Apply modern industrial dark theme"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a1a;
            }
            QWidget {
                background-color: #1a1a1a;
                color: #e0e0e0;
                font-family: 'Segoe UI', 'Arial', sans-serif;
                font-size: 11pt;
            }
            QFrame {
                background-color: #252525;
                border: 1px solid #3a3a3a;
                border-radius: 8px;
            }
            QPushButton {
                background-color: #2d2d2d;
                border: 2px solid #4a4a4a;
                border-radius: 6px;
                padding: 12px;
                color: #e0e0e0;
                font-weight: bold;
                font-size: 12pt;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
                border-color: #5a5a5a;
            }
            QPushButton:pressed {
                background-color: #1a1a1a;
            }
            QPushButton#powerBtn {
                background-color: #1a4d1a;
                border-color: #2d7a2d;
            }
            QPushButton#powerBtn:hover {
                background-color: #2d5a2d;
            }
            QPushButton#powerBtnOn {
                background-color: #2d7a2d;
                border-color: #3a9a3a;
            }
            QPushButton#stopBtn {
                background-color: #7a2d2d;
                border-color: #9a3a3a;
            }
            QPushButton#stopBtn:hover {
                background-color: #8a3a3a;
            }
            QPushButton#resetBtn {
                background-color: #7a5a2d;
                border-color: #9a7a3a;
            }
            QPushButton#resetBtn:hover {
                background-color: #8a6a3a;
            }
            QLineEdit {
                background-color: #2d2d2d;
                border: 2px solid #4a4a4a;
                border-radius: 4px;
                padding: 8px;
                color: #e0e0e0;
                font-size: 12pt;
            }
            QLineEdit:focus {
                border-color: #5a8ad4;
            }
            QSlider::groove:horizontal {
                background: #3a3a3a;
                height: 10px;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: #5a8ad4;
                width: 20px;
                margin: -5px 0;
                border-radius: 10px;
            }
            QSlider::handle:horizontal:hover {
                background: #6a9ae4;
            }
            QLabel {
                color: #e0e0e0;
                background: transparent;
                border: none;
            }
            QLabel#titleLabel {
                font-size: 16pt;
                font-weight: bold;
                color: #5a8ad4;
                border-bottom: 2px solid #5a8ad4;
                padding-bottom: 5px;
            }
            QLabel#valueLabel {
                font-size: 28pt;
                font-weight: bold;
                color: #4a9aff;
                font-family: 'Consolas', 'Courier New', monospace;
            }
            QLabel#unitLabel {
                font-size: 14pt;
                color: #8a8a8a;
            }
        """)

    def create_control_panel(self):
        """Create the left control panel"""
        panel = QFrame()
        layout = QVBoxLayout(panel)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)

        # Title
        title = QLabel("CONTROL PANEL")
        title.setObjectName("titleLabel")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Power button
        self.power_btn = QPushButton("POWER OFF")
        self.power_btn.setObjectName("powerBtn")
        self.power_btn.setMinimumHeight(60)
        self.power_btn.clicked.connect(self.toggle_power)
        layout.addWidget(self.power_btn)

        # Stop button
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("stopBtn")
        self.stop_btn.setMinimumHeight(60)
        self.stop_btn.clicked.connect(self.stop_treadmill)
        layout.addWidget(self.stop_btn)

        # Speed adjustment section
        speed_frame = QFrame()
        speed_layout = QVBoxLayout(speed_frame)

        speed_title = QLabel("Speed Adjustment")
        speed_title.setObjectName("titleLabel")
        speed_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        speed_layout.addWidget(speed_title)

        # Fine speed buttons (0.01 m/s)
        fine_layout = QHBoxLayout()
        self.fine_down_btn = QPushButton("◄ 0.01")
        self.fine_up_btn = QPushButton("0.01 ►")
        self.fine_down_btn.setMinimumHeight(50)
        self.fine_up_btn.setMinimumHeight(50)

        self.fine_down_btn.pressed.connect(lambda: self.start_speed_change(-0.01))
        self.fine_down_btn.released.connect(self.stop_speed_change)
        self.fine_up_btn.pressed.connect(lambda: self.start_speed_change(0.01))
        self.fine_up_btn.released.connect(self.stop_speed_change)

        fine_layout.addWidget(self.fine_down_btn)
        fine_layout.addWidget(self.fine_up_btn)
        speed_layout.addLayout(fine_layout)

        # Coarse speed buttons (0.1 m/s)
        coarse_layout = QHBoxLayout()
        self.coarse_down_btn = QPushButton("◄◄ 0.1")
        self.coarse_up_btn = QPushButton("0.1 ►►")
        self.coarse_down_btn.setMinimumHeight(50)
        self.coarse_up_btn.setMinimumHeight(50)

        self.coarse_down_btn.pressed.connect(lambda: self.start_speed_change(-0.1))
        self.coarse_down_btn.released.connect(self.stop_speed_change)
        self.coarse_up_btn.pressed.connect(lambda: self.start_speed_change(0.1))
        self.coarse_up_btn.released.connect(self.stop_speed_change)

        coarse_layout.addWidget(self.coarse_down_btn)
        coarse_layout.addWidget(self.coarse_up_btn)
        speed_layout.addLayout(coarse_layout)

        layout.addWidget(speed_frame)

        # Speed slider
        slider_frame = QFrame()
        slider_layout = QVBoxLayout(slider_frame)

        slider_title = QLabel("Speed Slider")
        slider_title.setObjectName("titleLabel")
        slider_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        slider_layout.addWidget(slider_title)

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(-200)  # -2.0 m/s
        self.speed_slider.setMaximum(200)  # +2.0 m/s
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setTickInterval(20)
        slider_layout.addWidget(self.speed_slider)

        self.slider_value_label = QLabel("0.00 m/s")
        self.slider_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_slider.valueChanged.connect(self.update_slider_label)
        slider_layout.addWidget(self.slider_value_label)

        self.set_slider_btn = QPushButton("SET SPEED FROM SLIDER")
        self.set_slider_btn.setMinimumHeight(50)
        self.set_slider_btn.clicked.connect(self.set_speed_from_slider)
        slider_layout.addWidget(self.set_slider_btn)

        layout.addWidget(slider_frame)

        # Manual speed entry
        entry_frame = QFrame()
        entry_layout = QVBoxLayout(entry_frame)

        entry_title = QLabel("Manual Speed Entry")
        entry_title.setObjectName("titleLabel")
        entry_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        entry_layout.addWidget(entry_title)

        self.speed_entry = QLineEdit()
        self.speed_entry.setPlaceholderText("Enter speed (m/s)")
        self.speed_entry.returnPressed.connect(self.set_speed_from_entry)
        entry_layout.addWidget(self.speed_entry)

        layout.addWidget(entry_frame)

        # Reset button
        self.reset_btn = QPushButton("RESET SYSTEM")
        self.reset_btn.setObjectName("resetBtn")
        self.reset_btn.setMinimumHeight(60)
        self.reset_btn.clicked.connect(self.reset_system)
        layout.addWidget(self.reset_btn)

        layout.addStretch()

        return panel

    def create_display_panel(self):
        """Create the right display panel"""
        panel = QFrame()
        layout = QVBoxLayout(panel)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)

        # Title
        title = QLabel("SYSTEM DISPLAY")
        title.setObjectName("titleLabel")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Speed display grid
        speed_grid = QGridLayout()
        speed_grid.setSpacing(10)

        # Current speed
        current_label = QLabel("Current Speed")
        current_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.current_speed_display = QLabel("0.00")
        self.current_speed_display.setObjectName("valueLabel")
        self.current_speed_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        current_unit = QLabel("m/s")
        current_unit.setObjectName("unitLabel")
        current_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)

        speed_grid.addWidget(current_label, 0, 0)
        speed_grid.addWidget(self.current_speed_display, 1, 0)
        speed_grid.addWidget(current_unit, 2, 0)

        # Set speed
        set_label = QLabel("Set Speed")
        set_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_speed_display = QLabel("0.00")
        self.set_speed_display.setObjectName("valueLabel")
        self.set_speed_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        set_unit = QLabel("m/s")
        set_unit.setObjectName("unitLabel")
        set_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)

        speed_grid.addWidget(set_label, 0, 1)
        speed_grid.addWidget(self.set_speed_display, 1, 1)
        speed_grid.addWidget(set_unit, 2, 1)

        layout.addLayout(speed_grid)

        # Stats display grid
        stats_grid = QGridLayout()
        stats_grid.setSpacing(10)

        # Distance
        dist_label = QLabel("Distance")
        dist_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.distance_display = QLabel("0.00")
        self.distance_display.setObjectName("valueLabel")
        self.distance_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        dist_unit = QLabel("m")
        dist_unit.setObjectName("unitLabel")
        dist_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)

        stats_grid.addWidget(dist_label, 0, 0)
        stats_grid.addWidget(self.distance_display, 1, 0)
        stats_grid.addWidget(dist_unit, 2, 0)

        # Time
        time_label = QLabel("Running Time")
        time_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.time_display = QLabel("00:00:00")
        self.time_display.setObjectName("valueLabel")
        self.time_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        time_unit = QLabel("h:m:s")
        time_unit.setObjectName("unitLabel")
        time_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)

        stats_grid.addWidget(time_label, 0, 1)
        stats_grid.addWidget(self.time_display, 1, 1)
        stats_grid.addWidget(time_unit, 2, 1)

        layout.addLayout(stats_grid)

        # Speed graph
        graph_title = QLabel("Speed History")
        graph_title.setObjectName("titleLabel")
        graph_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(graph_title)

        # Configure pyqtgraph for dark theme
        pg.setConfigOption("background", "#252525")
        pg.setConfigOption("foreground", "#e0e0e0")

        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setMinimumHeight(300)
        self.graph_widget.setLabel("left", "Speed", units="m/s")
        self.graph_widget.setLabel("bottom", "Time", units="s")
        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)
        self.graph_widget.setYRange(-2.5, 2.5)

        # Create plot line
        self.speed_plot = self.graph_widget.plot(pen=pg.mkPen(color="#4a9aff", width=2))

        layout.addWidget(self.graph_widget)

        return panel

    def update_slider_label(self, value):
        """Update the label showing slider value"""
        speed = value / 100.0
        self.slider_value_label.setText(f"{speed:.2f} m/s")

    def toggle_power(self):
        """Toggle treadmill power on/off"""
        if self.is_powered:
            # Power off
            self.ros_node.send_special_command("off")
            self.is_powered = False
            self.power_btn.setText("POWER OFF")
            self.power_btn.setObjectName("powerBtn")
            self.power_btn.setStyle(self.power_btn.style())  # Refresh style
            self.ui_timer.stop()
        else:
            # Power on
            self.ros_node.send_special_command("on")
            self.is_powered = True
            self.power_btn.setText("POWER ON")
            self.power_btn.setObjectName("powerBtnOn")
            self.power_btn.setStyle(self.power_btn.style())  # Refresh style

    def stop_treadmill(self):
        """Stop the treadmill and pause tracking"""
        self.ros_node.send_special_command("stop")
        self.is_running = False
        self.ui_timer.stop()

    def start_speed_change(self, delta):
        """Start changing speed with button press"""
        self.speed_hold_active = delta
        self.adjust_speed(delta)
        self.speed_hold_timer.start(200)  # Initial delay

    def stop_speed_change(self):
        """Stop changing speed on button release"""
        self.speed_hold_active = None
        self.speed_hold_timer.stop()

    def handle_speed_hold(self):
        """Handle held speed button (5x rate after initial press)"""
        if self.speed_hold_active is not None:
            self.adjust_speed(self.speed_hold_active * 5)
            self.speed_hold_timer.setInterval(1000)  # Continue at 1Hz

    def adjust_speed(self, delta):
        """Adjust the set speed by delta"""
        self.set_speed_mps = round(self.set_speed_mps + delta, 2)
        self.set_speed_mps = max(-2.0, min(2.0, self.set_speed_mps))  # Clamp
        self.set_speed_display.setText(f"{self.set_speed_mps:.2f}")

        # Send the speed command
        self.send_speed_command(self.set_speed_mps)

    def set_speed_from_slider(self):
        """Set speed from slider value"""
        speed = self.speed_slider.value() / 100.0
        self.set_speed_mps = speed
        self.set_speed_display.setText(f"{self.set_speed_mps:.2f}")
        self.send_speed_command(self.set_speed_mps)

    def set_speed_from_entry(self):
        """Set speed from manual entry"""
        try:
            speed = float(self.speed_entry.text())
            speed = max(-2.0, min(2.0, speed))  # Clamp
            self.set_speed_mps = speed
            self.set_speed_display.setText(f"{self.set_speed_mps:.2f}")
            self.speed_entry.clear()
            self.send_speed_command(self.set_speed_mps)
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid number")

    def send_speed_command(self, speed):
        """Send speed command and start if powered on"""
        self.ros_node.send_speed(speed)

        # If powered on and speed is not zero, send start command
        if self.is_powered and speed != 0:
            self.ros_node.send_special_command("start")
            if not self.is_running:
                self.is_running = True
                self.last_update_time = datetime.now()
                self.ui_timer.start()

    def reset_system(self):
        """Reset the system - turn off, reset counters, clear graph"""
        # Power off
        self.ros_node.send_special_command("off")
        self.is_powered = False
        self.power_btn.setText("POWER OFF")
        self.power_btn.setObjectName("powerBtn")
        self.power_btn.setStyle(self.power_btn.style())

        # Stop timer
        self.ui_timer.stop()
        self.is_running = False

        # Reset values
        self.set_speed_mps = 0.0
        self.current_speed_mps = 0.0
        self.distance_m = 0.0
        self.running_time = timedelta(0)
        self.last_update_time = None

        # Clear graph
        self.time_data.clear()
        self.speed_data.clear()
        self.speed_plot.setData([], [])

        # Update displays
        self.set_speed_display.setText("0.00")
        self.current_speed_display.setText("0.00")
        self.distance_display.setText("0.00")
        self.time_display.setText("00:00:00")

        # Send speed 0 and power on
        self.ros_node.send_speed(0.0)
        self.ros_node.send_special_command("on")
        self.is_powered = True
        self.power_btn.setText("POWER ON")
        self.power_btn.setObjectName("powerBtnOn")
        self.power_btn.setStyle(self.power_btn.style())

    def update_time_and_distance(self):
        """Update running time and distance calculations"""
        if not self.is_running or not self.last_update_time:
            return

        now = datetime.now()
        dt = (now - self.last_update_time).total_seconds()
        self.last_update_time = now

        # Update time
        self.running_time += timedelta(seconds=dt)
        hours = int(self.running_time.total_seconds() // 3600)
        minutes = int((self.running_time.total_seconds() % 3600) // 60)
        seconds = int(self.running_time.total_seconds() % 60)
        self.time_display.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

        # Update distance
        self.distance_m += abs(self.current_speed_mps) * dt
        self.distance_display.setText(f"{self.distance_m:.2f}")

    def handle_status_update(self, status):
        """Handle incoming status messages from ROS"""
        # Update current speed
        if status.speed_mps != -999999:
            self.current_speed_mps = status.speed_mps
            self.current_speed_display.setText(f"{self.current_speed_mps:.2f}")

            # Update graph
            if self.is_running and self.is_powered:
                elapsed = self.running_time.total_seconds()
                self.time_data.append(elapsed)
                self.speed_data.append(self.current_speed_mps)

                # Limit data points
                if len(self.time_data) > self.max_graph_points:
                    self.time_data.pop(0)
                    self.speed_data.pop(0)

                # Update plot
                self.speed_plot.setData(self.time_data, self.speed_data)

        # Check for errors
        if status.error:
            self.show_error_dialog(status.error)

    def show_error_dialog(self, error_code):
        """Show error dialog with code description"""
        if not error_code:
            return

        # Parse error code (format: "F123" or "A456")
        error_type = error_code[0]  # 'F' or 'A'
        try:
            # Extract numeric code, handling both integer and float formats
            code_str = error_code[1:]
            if "." in code_str:
                code_num = int(float(code_str))
            else:
                code_num = int(code_str)
        except (ValueError, IndexError):
            QMessageBox.critical(
                self,
                "Treadmill Error",
                f"Error code: {error_code}\n\nInvalid error code format.",
            )
            return

        # Get error information
        if error_type == "F":
            error_info = get_fault(code_num, description_only=False)
            icon = QMessageBox.Icon.Critical
            title = f"Treadmill Fault: F{code_num:03d}"
        elif error_type == "A":
            error_info = get_alarm(code_num, description_only=False)
            icon = QMessageBox.Icon.Warning
            title = f"Treadmill Alarm: A{code_num:03d}"
        else:
            QMessageBox.critical(
                self,
                "Treadmill Error",
                f"Error code: {error_code}\n\nUnknown error type.",
            )
            return

        # Parse the error info string
        lines = error_info.split("\n")
        description = ""
        causes = ""
        for line in lines:
            if line.startswith("Description:"):
                description = line.replace("Description:", "").strip()
            elif line.startswith("Possible Causes:"):
                causes = line.replace("Possible Causes:", "").strip()

        # Create message box
        msg = QMessageBox(self)
        msg.setIcon(icon)
        msg.setWindowTitle(title)
        msg.setText(f"<b>{description}</b>")
        msg.setInformativeText(f"<b>Possible Causes:</b><br>{causes}")
        msg.setStandardButtons(QMessageBox.StandardButton.Ok)

        # Add reset fault button for faults
        if error_type == "F":
            reset_btn = msg.addButton("Reset Fault", QMessageBox.ButtonRole.ActionRole)
            reset_btn.clicked.connect(self.reset_fault)

        msg.exec()

    def reset_fault(self):
        """Send reset fault command"""
        self.ros_node.send_special_command("reset_fault")


def ros_spin_thread(node):
    """Run ROS spin in separate thread"""
    rclpy.spin(node)


def main():
    """Main entry point"""
    # Initialize ROS
    rclpy.init()

    # Create Qt application
    app = QApplication(sys.argv)

    # Create signal bridge
    signals = ROSSignals()

    # Create ROS node
    ros_node = TreadmillGUINode(signals)

    # Create GUI
    gui = TreadmillGUI(ros_node)

    # Connect signals
    signals.status_updated.connect(gui.handle_status_update)

    # Start ROS spinning in separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Run Qt event loop
    exit_code = app.exec()

    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
