#!/usr/bin/env python3

# switch_sensor_node.py
# ──────────────────────────────────────────────────────────────────────────────
# - Lytter på en end-switch (GPIO27)
# - publicerer "up" eller "released" på /limit_state.
# ──────────────────────────────────────────────────────────────────────────────

# ─── Imports ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Button

# ────────────────────────────────────────────────────────────────────────────────
class SwitchSensorNode(Node):
    def __init__(self):
        super().__init__("switch_sensor_node")
        self.pub = self.create_publisher(msg=String, topic="/limit_state", qos=10)

        # ── end-switch: aktiv (pressed) = "up"───────────────────────────────────
        self.switch = Button(27, pull_up=True, bounce_time=0.02)
        self.switch.when_pressed   = lambda: self.publish("up")
        self.switch.when_released  = lambda: self.publish("released")
        self.get_logger().info("Switch-sensor klar på GPIO 27.")

    # ─────publish state──────────────────────────────────────────────────────────
    def publish(self, state=str):
        self.pub.publish(String(data=state))
        self.get_logger().info(f"/limit_state: {state}")

# ──── Main-indgang ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = SwitchSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node() 
        rclpy.shutdown()


if __name__ == "__main__":
    main()
