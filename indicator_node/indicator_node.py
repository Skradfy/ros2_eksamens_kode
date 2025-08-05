#!/usr/bin/env python3

# signal_node.py
# ──────────────────────────────────────────────────────────────────────────────
# - Venter på trykknap (GPIO16)
# - publicerer std_msgs/Empty på topic '/signal', når knappen trykkes ind
# ──────────────────────────────────────────────────────────────────────────────

# ─── Imports ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from gpiozero import Button

# ──────────────────────────────────────────────────────────────────────────────
class SignalNode(Node):
    def __init__(self):
        super().__init__("signal_node")

        # ─── hardwareopsætning ────────────────────────────────────────────────
        self.button = Button(GPIO=16, 
                             pull_up=True, 
                             debounce_time=0.05)

        # ─── publisher ────────────────────────────────────────────────────────
        self.pub = self.create_publisher(msg=Empty, topic="/signal", 
                                                    qos_profile=10)

        # ─── subscriber til reset ─────────────────────────────────────────────
        self.create_subscription(msg=Empty, topic="/signal_reset", 
                                            callback=self.cb_reset, 
                                            qos_profile=10)

        # ─── registrér callback når knappen trykkes ───────────────────────────
        self.button.when_pressed = self.handle_press
        self.get_logger().info(" signal_node klar, tryk på knappen!")

    # ───── Udsend ét Empty-msg ved knaptryk ────────────────────────────────────
    def handle_press(self) -> None:
        self.pub.publish(Empty())
        self.get_logger().info("Signal sendt.")

    def cb_reset(self, msg=Empty) -> None:
        self.get_logger().info("Reset modtaget – klar til nyt knaptryk.")

# ─── Main-indgang ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = SignalNode()
    try:
        rclpy.spin(node) # kør, indtil Ctrl-C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
