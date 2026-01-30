#!/usr/bin/env python3
"""
Treadmill GUI - Enhanced Aesthetics Version
A modern, industrial-themed interface for treadmill control
"""

import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QGridLayout, QMessageBox, QGroupBox,
    QFrame, QGraphicsDropShadowEffect
)
from PyQt6.QtCore import QTimer, Qt, QPropertyAnimation, QEasingCurve, QRect, pyqtProperty
from PyQt6.QtGui import QFont, QPalette, QColor, QPainter, QPen
import pyqtgraph as pg
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from treadmill_interfaces.msg import TreadmillStatus


# Custom styled frame with gradient background
class GradientFrame(QFrame):
    """Frame with custom gradient background"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAutoFillBackground(True)
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Create subtle gradient
        from PyQt6.QtGui import QLinearGradient
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor(30, 33, 38))
        gradient.setColorAt(1, QColor(24, 27, 31))
        
        painter.fillRect(self.rect(), gradient)
        painter.end()


class AnimatedButton(QPushButton):
    """Button with smooth animations and glow effects"""
    
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self._glow_intensity = 0
        
        # Add shadow effect for depth
        self.shadow = QGraphicsDropShadowEffect()
        self.shadow.setBlurRadius(15)
        self.shadow.setColor(QColor(0, 0, 0, 100))
        self.shadow.setOffset(0, 2)
        self.setGraphicsEffect(self.shadow)
        
        # Animation for hover effects
        self.anim = QPropertyAnimation(self, b"glow_intensity")
        self.anim.setDuration(200)
        self.anim.setEasingCurve(QEasingCurve.Type.OutCubic)
    
    def get_glow_intensity(self):
        return self._glow_intensity
    
    def set_glow_intensity(self, value):
        self._glow_intensity = value
        self.update()
    
    glow_intensity = pyqtProperty(int, get_glow_intensity, set_glow_intensity)
    
    def enterEvent(self, event):
        self.anim.setStartValue(self._glow_intensity)
        self.anim.setEndValue(100)
        self.anim.start()
        super().enterEvent(event)
    
    def leaveEvent(self, event):
        self.anim.setStartValue(self._glow_intensity)
        self.anim.setEndValue(0)
        self.anim.start()
        super().leaveEvent(event)


class TreadmillGUI(QMainWindow):
    """Main GUI window with enhanced aesthetics"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Treadmill Control System")
        self.setGeometry(100, 100, 1400, 900)
        
        # State variables
        self.power_on = False
        self.current_speed = 0.0
        self.set_speed = 0.0
        self.distance = 0.0
        self.elapsed_time = 0.0
        self.is_running = False
        self.last_error = ""
        
        # Data for graph
        self.max_data_points = 500
        self.time_data = deque(maxlen=self.max_data_points)
        self.speed_data = deque(maxlen=self.max_data_points)
        
        # Button hold state
        self.speed_button_held = None
        self.speed_button_timer = QTimer()
        self.speed_button_timer.timeout.connect(self.handle_speed_button_hold)
        
        # Initialize ROS2
        try:
            rclpy.init()
            self.ros_node = TreadmillROSNode(self)
        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")
            self.ros_node = None
        
        # Apply modern dark theme
        self.apply_stylesheet()
        
        # Setup UI
        self.init_ui()
        
        # Timers
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)
        
        self.time_timer = QTimer()
        self.time_timer.timeout.connect(self.update_time_and_distance)
        self.time_timer.start(100)
    
    def apply_stylesheet(self):
        """Apply comprehensive modern stylesheet"""
        stylesheet = """
            QMainWindow {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #1e2126, stop:1 #181b1f
                );
            }
            
            QGroupBox {
                color: #e0e0e0;
                border: 2px solid #2d3238;
                border-radius: 12px;
                margin-top: 12px;
                padding-top: 20px;
                font-family: 'Segoe UI', 'SF Pro Display', system-ui;
                font-size: 13px;
                font-weight: 600;
                letter-spacing: 0.5px;
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(45, 50, 56, 0.4), 
                    stop:1 rgba(35, 39, 44, 0.4)
                );
            }
            
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 4px 12px;
                color: #00d9ff;
                background: rgba(0, 217, 255, 0.1);
                border-radius: 6px;
                margin-left: 10px;
            }
            
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3a4149, stop:1 #2d3238
                );
                color: #ffffff;
                border: 2px solid #4a5259;
                border-radius: 10px;
                padding: 12px 24px;
                font-family: 'Segoe UI Semibold', 'SF Pro Display', system-ui;
                font-size: 14px;
                font-weight: 600;
                letter-spacing: 0.8px;
                text-transform: uppercase;
            }
            
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #4a5259, stop:1 #3a4149
                );
                border: 2px solid #5a6269;
            }
            
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2d3238, stop:1 #252930
                );
                padding-top: 14px;
                padding-bottom: 10px;
            }
            
            QPushButton#powerButton {
                font-size: 16px;
                font-weight: 700;
                min-height: 65px;
            }
            
            QPushButton#stopButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #d32f2f, stop:1 #b71c1c
                );
                border: 2px solid #f44336;
                font-size: 16px;
                font-weight: 700;
                min-height: 65px;
            }
            
            QPushButton#stopButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #e53935, stop:1 #c62828
                );
                border: 2px solid #ff5252;
            }
            
            QPushButton#resetButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #ff6f00, stop:1 #e65100
                );
                border: 2px solid #ff9800;
                min-height: 65px;
            }
            
            QPushButton#resetButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #ff8f00, stop:1 #ef6c00
                );
                border: 2px solid #ffab00;
            }
            
            QPushButton#speedButton {
                min-height: 55px;
                font-size: 20px;
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #00acc1, stop:1 #00838f
                );
                border: 2px solid #00bcd4;
            }
            
            QPushButton#speedButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #00bcd4, stop:1 #0097a7
                );
                border: 2px solid #00e5ff;
            }
            
            QLineEdit {
                background: #252930;
                color: #ffffff;
                border: 2px solid #3a4149;
                border-radius: 8px;
                padding: 12px 16px;
                font-family: 'Consolas', 'SF Mono', 'Monaco', monospace;
                font-size: 15px;
                selection-background-color: #00d9ff;
            }
            
            QLineEdit:focus {
                border: 2px solid #00d9ff;
                background: #2d3238;
            }
            
            QLabel {
                color: #e0e0e0;
                font-family: 'Segoe UI', 'SF Pro Display', system-ui;
            }
            
            QLabel#valueLabel {
                color: #00d9ff;
                font-family: 'Consolas', 'SF Mono', monospace;
                font-size: 32px;
                font-weight: 700;
                letter-spacing: 1px;
                padding: 8px;
                background: rgba(0, 217, 255, 0.08);
                border-radius: 8px;
                border-left: 4px solid #00d9ff;
            }
            
            QLabel#labelText {
                color: #9e9e9e;
                font-size: 12px;
                font-weight: 600;
                text-transform: uppercase;
                letter-spacing: 1.2px;
            }
        """
        self.setStyleSheet(stylesheet)
    
    def init_ui(self):
        """Initialize the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Left panel
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Right panel
        right_panel = self.create_display_panel()
        main_layout.addWidget(right_panel, 2)
    
    def create_control_panel(self):
        """Create enhanced control panel"""
        panel = QGroupBox("CONTROL PANEL")
        layout = QVBoxLayout()
        layout.setSpacing(16)
        
        # Power controls
        power_layout = QHBoxLayout()
        power_layout.setSpacing(12)
        
        self.on_off_button = AnimatedButton("⚡ POWER OFF")
        self.on_off_button.setObjectName("powerButton")
        self.on_off_button.setStyleSheet("""
            QPushButton#powerButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #c62828, stop:1 #b71c1c);
                border: 2px solid #e53935;
            }
            QPushButton#powerButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #d32f2f, stop:1 #c62828);
                border: 2px solid #f44336;
            }
        """)
        self.on_off_button.clicked.connect(self.toggle_power)
        power_layout.addWidget(self.on_off_button)
        
        self.reset_button = AnimatedButton("↻ RESET")
        self.reset_button.setObjectName("resetButton")
        self.reset_button.clicked.connect(self.reset_treadmill)
        power_layout.addWidget(self.reset_button)
        layout.addLayout(power_layout)
        
        # Stop button
        self.stop_button = AnimatedButton("■ STOP")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.clicked.connect(self.stop_treadmill)
        layout.addWidget(self.stop_button)
        
        # Separator line
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("background: #3a4149; min-height: 2px; margin: 12px 0;")
        layout.addWidget(line)
        
        # Speed controls
        speed_group = QGroupBox("SPEED ADJUSTMENT")
        speed_layout = QVBoxLayout()
        speed_layout.setSpacing(12)
        
        # Fine control
        fine_layout = QVBoxLayout()
        fine_label = QLabel("FINE CONTROL")
        fine_label.setObjectName("labelText")
        fine_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        fine_layout.addWidget(fine_label)
        
        fine_buttons = QHBoxLayout()
        fine_buttons.setSpacing(8)
        
        self.fine_down_button = AnimatedButton("▼")
        self.fine_down_button.setObjectName("speedButton")
        self.fine_down_button.pressed.connect(lambda: self.start_speed_change(-0.01))
        self.fine_down_button.released.connect(self.stop_speed_change)
        fine_buttons.addWidget(self.fine_down_button)
        
        fine_value = QLabel("± 0.01 m/s")
        fine_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        fine_value.setStyleSheet("font-size: 13px; font-weight: 600; color: #00d9ff;")
        fine_buttons.addWidget(fine_value)
        
        self.fine_up_button = AnimatedButton("▲")
        self.fine_up_button.setObjectName("speedButton")
        self.fine_up_button.pressed.connect(lambda: self.start_speed_change(0.01))
        self.fine_up_button.released.connect(self.stop_speed_change)
        fine_buttons.addWidget(self.fine_up_button)
        
        fine_layout.addLayout(fine_buttons)
        speed_layout.addLayout(fine_layout)
        
        # Coarse control
        coarse_layout = QVBoxLayout()
        coarse_label = QLabel("COARSE CONTROL")
        coarse_label.setObjectName("labelText")
        coarse_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        coarse_layout.addWidget(coarse_label)
        
        coarse_buttons = QHBoxLayout()
        coarse_buttons.setSpacing(8)
        
        self.coarse_down_button = AnimatedButton("▼▼")
        self.coarse_down_button.setObjectName("speedButton")
        self.coarse_down_button.pressed.connect(lambda: self.start_speed_change(-0.1))
        self.coarse_down_button.released.connect(self.stop_speed_change)
        coarse_buttons.addWidget(self.coarse_down_button)
        
        coarse_value = QLabel("± 0.1 m/s")
        coarse_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        coarse_value.setStyleSheet("font-size: 13px; font-weight: 600; color: #00d9ff;")
        coarse_buttons.addWidget(coarse_value)
        
        self.coarse_up_button = AnimatedButton("▲▲")
        self.coarse_up_button.setObjectName("speedButton")
        self.coarse_up_button.pressed.connect(lambda: self.start_speed_change(0.1))
        self.coarse_up_button.released.connect(self.stop_speed_change)
        coarse_buttons.addWidget(self.coarse_up_button)
        
        coarse_layout.addLayout(coarse_buttons)
        speed_layout.addLayout(coarse_layout)
        
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        
        # Manual entry
        manual_group = QGroupBox("DIRECT INPUT")
        manual_layout = QVBoxLayout()
        manual_layout.setSpacing(8)
        
        input_layout = QHBoxLayout()
        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("Enter speed value...")
        self.speed_input.returnPressed.connect(self.set_manual_speed)
        input_layout.addWidget(self.speed_input)
        
        set_button = AnimatedButton("SET")
        set_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #43a047, stop:1 #2e7d32);
                border: 2px solid #66bb6a;
                min-width: 80px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #4caf50, stop:1 #388e3c);
                border: 2px solid #81c784;
            }
        """)
        set_button.clicked.connect(self.set_manual_speed)
        input_layout.addWidget(set_button)
        
        manual_layout.addLayout(input_layout)
        
        hint = QLabel("Accepts positive/negative values")
        hint.setStyleSheet("font-size: 10px; color: #757575; font-style: italic;")
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        manual_layout.addWidget(hint)
        
        manual_group.setLayout(manual_layout)
        layout.addWidget(manual_group)
        
        layout.addStretch()
        panel.setLayout(layout)
        return panel
    
    def create_display_panel(self):
        """Create enhanced display panel"""
        panel = QGroupBox("SYSTEM MONITOR")
        layout = QVBoxLayout()
        layout.setSpacing(16)
        
        # Status displays at top
        status_grid = QGridLayout()
        status_grid.setSpacing(16)
        status_grid.setHorizontalSpacing(20)
        
        # Current speed
        current_label = QLabel("CURRENT SPEED")
        current_label.setObjectName("labelText")
        status_grid.addWidget(current_label, 0, 0)
        
        self.current_speed_label = QLabel("0.00")
        self.current_speed_label.setObjectName("valueLabel")
        self.current_speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(self.current_speed_label, 1, 0)
        
        speed_unit = QLabel("m/s")
        speed_unit.setStyleSheet("color: #757575; font-size: 11px; font-weight: 600;")
        speed_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(speed_unit, 2, 0)
        
        # Set speed
        set_label = QLabel("TARGET SPEED")
        set_label.setObjectName("labelText")
        status_grid.addWidget(set_label, 0, 1)
        
        self.set_speed_label = QLabel("0.00")
        self.set_speed_label.setObjectName("valueLabel")
        self.set_speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(self.set_speed_label, 1, 1)
        
        set_unit = QLabel("m/s")
        set_unit.setStyleSheet("color: #757575; font-size: 11px; font-weight: 600;")
        set_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(set_unit, 2, 1)
        
        # Distance
        dist_label = QLabel("DISTANCE")
        dist_label.setObjectName("labelText")
        status_grid.addWidget(dist_label, 0, 2)
        
        self.distance_label = QLabel("0.00")
        self.distance_label.setObjectName("valueLabel")
        self.distance_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(self.distance_label, 1, 2)
        
        dist_unit = QLabel("meters")
        dist_unit.setStyleSheet("color: #757575; font-size: 11px; font-weight: 600;")
        dist_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(dist_unit, 2, 2)
        
        # Time
        time_label = QLabel("ELAPSED TIME")
        time_label.setObjectName("labelText")
        status_grid.addWidget(time_label, 0, 3)
        
        self.time_label = QLabel("00:00:00")
        self.time_label.setObjectName("valueLabel")
        self.time_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(self.time_label, 1, 3)
        
        time_unit = QLabel("HH:MM:SS")
        time_unit.setStyleSheet("color: #757575; font-size: 11px; font-weight: 600;")
        time_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        status_grid.addWidget(time_unit, 2, 3)
        
        layout.addLayout(status_grid)
        
        # Graph
        graph_group = QGroupBox("SPEED PROFILE")
        graph_layout = QVBoxLayout()
        
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#1e2126')
        self.plot_widget.setLabel('left', 'Speed', units='m/s', 
                                  color='#00d9ff', **{'font-size': '12pt'})
        self.plot_widget.setLabel('bottom', 'Time', units='s',
                                  color='#00d9ff', **{'font-size': '12pt'})
        
        # Style the plot
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.getAxis('left').setPen('#3a4149')
        self.plot_widget.getAxis('bottom').setPen('#3a4149')
        self.plot_widget.getAxis('left').setTextPen('#9e9e9e')
        self.plot_widget.getAxis('bottom').setTextPen('#9e9e9e')
        
        # Create gradient pen for speed curve
        gradient_color = pg.mkColor('#00d9ff')
        pen = pg.mkPen(color=gradient_color, width=3)
        self.speed_curve = self.plot_widget.plot(pen=pen)
        
        # Add fill under curve
        self.fill = self.plot_widget.plot(fillLevel=0, 
                                          brush=pg.mkBrush(0, 217, 255, 30))
        
        graph_layout.addWidget(self.plot_widget)
        graph_group.setLayout(graph_layout)
        layout.addWidget(graph_group)
        
        panel.setLayout(layout)
        return panel
    
    def toggle_power(self):
        """Toggle power with smooth transition"""
        if self.ros_node:
            if self.power_on:
                self.ros_node.send_command("power_off")
                self.power_on = False
                self.on_off_button.setText("⚡ POWER OFF")
                self.on_off_button.setStyleSheet("""
                    QPushButton#powerButton {
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #c62828, stop:1 #b71c1c);
                        border: 2px solid #e53935;
                    }
                    QPushButton#powerButton:hover {
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #d32f2f, stop:1 #c62828);
                        border: 2px solid #f44336;
                    }
                """)
            else:
                self.ros_node.send_command("power_on")
                self.power_on = True
                self.on_off_button.setText("⚡ POWER ON")
                self.on_off_button.setStyleSheet("""
                    QPushButton#powerButton {
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #43a047, stop:1 #2e7d32);
                        border: 2px solid #66bb6a;
                    }
                    QPushButton#powerButton:hover {
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #4caf50, stop:1 #388e3c);
                        border: 2px solid #81c784;
                    }
                """)
    
    def stop_treadmill(self):
        """Send stop command"""
        if self.ros_node:
            self.ros_node.send_command("stop")
            self.set_speed = 0.0
            self.is_running = False
    
    def reset_treadmill(self):
        """Reset treadmill"""
        if self.ros_node:
            self.ros_node.send_command("power_off")
            self.power_on = False
            
            self.set_speed = 0.0
            self.distance = 0.0
            self.elapsed_time = 0.0
            self.is_running = False
            
            self.time_data.clear()
            self.speed_data.clear()
            
            # Clear the graph display immediately
            self.speed_curve.setData([], [])
            self.fill.setData([], [])
            
            QTimer.singleShot(500, lambda: self.ros_node.send_command("power_on"))
            QTimer.singleShot(500, lambda: setattr(self, 'power_on', True))
            
            self.on_off_button.setText("⚡ POWER ON")
            self.on_off_button.setStyleSheet("""
                QPushButton#powerButton {
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 #43a047, stop:1 #2e7d32);
                    border: 2px solid #66bb6a;
                }
                QPushButton#powerButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 #4caf50, stop:1 #388e3c);
                    border: 2px solid #81c784;
                }
            """)
    
    def start_speed_change(self, increment):
        """Start speed change"""
        self.speed_button_held = increment
        self.change_speed(increment)
        self.speed_button_timer.start(200)
    
    def stop_speed_change(self):
        """Stop speed change"""
        self.speed_button_held = None
        self.speed_button_timer.stop()
    
    def handle_speed_button_hold(self):
        """Handle held button"""
        if self.speed_button_held is not None:
            self.change_speed(self.speed_button_held * 5)
    
    def change_speed(self, increment):
        """Change speed by increment"""
        self.set_speed += increment
        self.set_speed = round(self.set_speed, 2)
        
        if self.ros_node:
            self.ros_node.send_speed(self.set_speed)
        
        if self.set_speed != 0.0 and self.power_on:
            self.is_running = True
    
    def set_manual_speed(self):
        """Set manual speed"""
        try:
            speed = float(self.speed_input.text())
            self.set_speed = speed
            
            if self.ros_node:
                self.ros_node.send_speed(self.set_speed)
            
            self.speed_input.clear()
            
            if self.set_speed != 0.0 and self.power_on:
                self.is_running = True
                
        except ValueError:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Icon.Warning)
            msg.setWindowTitle("Invalid Input")
            msg.setText("Please enter a valid numeric value")
            msg.setStyleSheet("""
                QMessageBox {
                    background: #2d3238;
                }
                QLabel {
                    color: #e0e0e0;
                    font-size: 13px;
                }
                QPushButton {
                    min-width: 80px;
                }
            """)
            msg.exec()
    
    def update_time_and_distance(self):
        """Update time and distance"""
        if self.is_running and self.power_on and self.set_speed != 0:
            self.elapsed_time += 0.1
            self.distance += abs(self.current_speed) * 0.1
    
    def update_display(self):
        """Update display elements"""
        # Update numeric displays
        self.current_speed_label.setText(f"{self.current_speed:.2f}")
        self.set_speed_label.setText(f"{self.set_speed:.2f}")
        self.distance_label.setText(f"{self.distance:.2f}")
        
        hours = int(self.elapsed_time // 3600)
        minutes = int((self.elapsed_time % 3600) // 60)
        seconds = int(self.elapsed_time % 60)
        self.time_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        
        # Update graph
        if self.is_running and self.power_on:
            self.time_data.append(self.elapsed_time)
            self.speed_data.append(self.current_speed)
            
            if len(self.time_data) > 0:
                time_list = list(self.time_data)
                speed_list = list(self.speed_data)
                self.speed_curve.setData(time_list, speed_list)
                self.fill.setData(time_list, speed_list)
        
        if self.ros_node:
            rclpy.spin_once(self.ros_node, timeout_sec=0.01)
    
    def update_from_ros(self, speed, error):
        """Update from ROS messages"""
        self.current_speed = speed
        
        if error and error != self.last_error:
            self.last_error = error
            self.show_error_popup(error)
    
    def show_error_popup(self, error_code):
        """Show styled error popup"""
        if error_code.startswith('F'):
            code_num = int(error_code[1:])
            title = f"⚠ FAULT {error_code}"
            message = self.get_fault_info(code_num)
            icon = QMessageBox.Icon.Critical
        elif error_code.startswith('A'):
            code_num = int(error_code[1:])
            title = f"⚠ ALARM {error_code}"
            message = self.get_alarm_info(code_num)
            icon = QMessageBox.Icon.Warning
        else:
            title = "⚠ ERROR"
            message = f"Unknown error: {error_code}"
            icon = QMessageBox.Icon.Warning
        
        msg_box = QMessageBox(self)
        msg_box.setIcon(icon)
        msg_box.setWindowTitle(title)
        msg_box.setText(message)
        msg_box.setStyleSheet("""
            QMessageBox {
                background: #2d3238;
            }
            QLabel {
                color: #e0e0e0;
                font-size: 13px;
                min-width: 400px;
            }
            QPushButton {
                min-width: 100px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3a4149, stop:1 #2d3238);
                border: 2px solid #4a5259;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #4a5259, stop:1 #3a4149);
            }
        """)
        msg_box.exec()
    
    def get_fault_info(self, code):
        """Get fault information"""
        faults = {
            6: ("Imbalance or Input Phase Loss", 
                "Mains voltage imbalance too high; phase missing at input; input voltage imbalance > 5%."),
            21: ("DC Link Undervoltage", 
                 "Input voltage too low; Phase loss; Pre-charge circuit failure; P0296 set too high."),
            22: ("DC Link Overvoltage", 
                 "Inertia of driven-load too high; Deceleration time too short; P0151/P0153/P0185 set too high."),
            30: ("Power Module U Fault", 
                 "IGBT desaturation; Short-circuit between phases U-V or U-W."),
            34: ("Power Module V Fault", 
                 "IGBT desaturation; Short-circuit between phases V-U or V-W."),
            38: ("Power Module W Fault", 
                 "IGBT desaturation; Short-circuit between phases W-U or W-V."),
            42: ("DB IGBT Fault", 
                 "Desaturation of Dynamic Braking IGBT; Short-circuit in braking resistor cables."),
            48: ("IGBT Overload Fault", 
                 "High current at inverter output; check switching frequency vs. load."),
            51: ("IGBT Overtemperature", 
                 "High temperature detected by NTC sensors on IGBTs."),
            71: ("Output Overcurrent", 
                 "Excessive load inertia; Acceleration time too short; P0135/P0169/P0170 too high."),
            72: ("Motor Overload", 
                 "P0156/P0157/P0158 settings too low; Motor shaft load is excessive."),
            74: ("Ground Fault", 
                 "Short circuit to ground in motor/cables; Motor cable capacitance too large."),
            78: ("Motor Overtemperature", 
                 "Excessive motor load/duty cycle; Ambient temp too high; PTC wiring issues."),
            156: ("Undertemperature", 
                  "Surrounding air temperature <= -30 °C (-22 °F)."),
            185: ("Pre-charge Contactor Fault", 
                  "Open command fuse; Phase loss in L1/R or L2/S; Contactor defect.")
        }
        
        info = faults.get(code, ("Unknown Fault", "No information available"))
        return f"Description: {info[0]}\n\nPossible Causes:\n{info[1]}"
    
    def get_alarm_info(self, code):
        """Get alarm information"""
        alarms = {
            46: ("High Load on Motor", 
                 "Settings of P0156/P0157/P0158 too low; Motor shaft load excessive."),
            47: ("IGBT Overload Alarm", 
                 "High current at inverter output; Check switching frequency table."),
            50: ("IGBT High Temperature", 
                 "High ambient temp (> 45°C); Fan blocked/defective; Dirty heatsink."),
            88: ("Communication Lost", 
                 "Loose keypad cable; Electrical noise in installation."),
            90: ("External Alarm", 
                 "Digital input set to 'No External Alarm' is open/unwired."),
            110: ("High Motor Temperature", 
                  "Excessive shaft load; High ambient temp; PTC sensor wiring issues."),
            128: ("Timeout for Serial Communication", 
                  "No valid messages received within P0314 interval; Check wiring/grounding."),
            152: ("Internal Air High Temperature", 
                  "Ambient temp > 45°C; Internal fan defective."),
            177: ("Fan Replacement", 
                  "Heatsink fan has exceeded 50,000 operating hours."),
            702: ("Inverter Disabled", 
                  "General Enable command not active while SoftPLC is in Run mode.")
        }
        
        info = alarms.get(code, ("Unknown Alarm", "No information available"))
        return f"Description: {info[0]}\n\nPossible Causes:\n{info[1]}"
    
    def closeEvent(self, event):
        """Clean up on close"""
        if self.ros_node:
            self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


class TreadmillROSNode(Node):
    """ROS2 node for treadmill communication"""
    
    def __init__(self, gui):
        super().__init__('treadmill_gui_node')
        self.gui = gui
        
        self.speed_pub = self.create_publisher(Float32, 'treadmill/cmd_speed_mps', 10)
        self.cmd_pub = self.create_publisher(String, 'treadmill/special_cmd', 10)
        
        self.status_sub = self.create_subscription(
            TreadmillStatus,
            'treadmill/status',
            self.status_callback,
            10
        )
    
    def send_speed(self, speed):
        """Send speed command"""
        msg = Float32()
        msg.data = float(speed)
        self.speed_pub.publish(msg)
    
    def send_command(self, command):
        """Send special command"""
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
    
    def status_callback(self, msg):
        """Handle status messages"""
        self.gui.update_from_ros(msg.speed_mps, msg.error)


def main():
    """Main entry point"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    gui = TreadmillGUI()
    gui.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
