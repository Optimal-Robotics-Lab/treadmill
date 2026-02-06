import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from pymodbus.client import ModbusSerialClient
from treadmill_interfaces.msg import TreadmillStatus
from gpiozero import LED
import requests
import time

# Replace with actual GPIO pin numbers for the LEDs
LED_POWER = LED(17)  # GPIO pin for power status LED
LED_INTERNET = LED(27)  # GPIO pin for internet status LED
LED_NODE = LED(22)  # GPIO pin for ROS node status LED
LED_TOPIC = LED(5)  # GPIO pin for topic activity LED


class StatusMonitor(Node):
    def __init__(self):
        super().__init__("status_monitor")

        # timestamps for last received messages
        self.last_msg_time = 0
        self.target_node = "/treadmill_node"

        # Subscription to treadmill cmd topics
        self.create_subscription(
            String, "treadmill/special_cmd", self.topic_callback, 10
        )
        self.create_subscription(
            Float32, "treadmill/cmd_speed_mps", self.topic_callback, 10
        )

        def topic_callback(self, msg):
            # Update timestamp of last received message
            self.last_msg_time = time.time()

        def check_internet(self):
            try:
                requests.get("http://www.google.com", timeout=1)
                return True
            except requests.RequestException:
                return False

        def check_node(self):
            nodes = self.get_node_names()
            return self.target_node in nodes


def main():
    rclpy.init()
    monitor = StatusMonitor()

    LED_POWER.on()  # Assume power is on if this script is running

    # TODO potentially write a seperate script for power status

    try:
        while rclpy.ok():
            # 1. Update internet status LED
            LED_INTERNET.on() if monitor.check_internet() else LED_INTERNET.off()

            # 2. Update ROS node status LED
            LED_NODE.on() if monitor.check_node() else LED_NODE.off()

            # 3. Update topic activity LED based on message reception
            if (
                time.time() - monitor.last_msg_time
            ) < 30:  # If a message was received in the last 30 seconds
                LED_TOPIC.on()
            else:
                LED_TOPIC.off()

            rclpy.spin_once(monitor, timeout_sec=1)
            time.sleep(2)

    except KeyboardInterrupt:
        pass

    finally:
        LED_POWER.off()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
