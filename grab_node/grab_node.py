#!/usr/bin/env python3
"""
grab_node.py
------------
• /grab_cmd        ("close" | "open")
• /grab_state      ("closed" | "opened" | "fault")
• /sequence_done   (Empty) sendes når griberen er åbnet igen
• Overstrøms-watchdog på /motor_current (volt fra CMS)
"""

import rclpy, board, busio
from rclpy.node import Node
from std_msgs.msg import String, Float32, Empty
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from functools import partial

class GrabNode(Node):
    # ─── konstanter ──────────────────────────────────────────
    PCA_CHANNEL   = 1
    SERVO_FREQ    = 50
    ANGLE_OPEN    = 20
    ANGLE_CLOSE   = 158
    MOVE_TIME_S   = 1.0                # tid før status-publish

    VOLT_PER_AMP  = 3.83               # 0,22 Ω × 17,41
    MAX_CURRENT   = 0.80               # A
    MAX_VOLT      = MAX_CURRENT * VOLT_PER_AMP   # ≈ 3,06 V
    TRIP_SAMPLES  = 5
    # --------------------------------------------------------

    def __init__(self):
        super().__init__("grab_node")

        # hardware
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c); self.pca.frequency = self.SERVO_FREQ
        self.servo = servo.Servo(self.pca.channels[self.PCA_CHANNEL],
                                 min_pulse=500, max_pulse=2700)

        # pub/sub
        self.state_pub = self.create_publisher(String, "/grab_state", 10)
        self.done_pub  = self.create_publisher(Empty,  "/sequence_done", 10)

        self.create_subscription(String,  "/grab_cmd",      self.cmd_cb, 10)
        self.create_subscription(Float32, "/motor_current", self.cb_cur, 10)

        self.state    = "idle"
        self.over_cnt = 0
        self.get_logger().info("Grab-node klar – venter på /grab_cmd.")

    # ─── kommandoer ─────────────────────────────────────────
    def cmd_cb(self, msg: String):
        cmd = msg.data.lower().strip()
        if cmd == "close":
            self._start_move(self.ANGLE_CLOSE, "closed", next_state="closing")
        elif cmd == "open":
            self._start_move(self.ANGLE_OPEN,  "opened", next_state="opening")
        else:
            self.get_logger().warn(f"Ugyldig /grab_cmd: {cmd}")

    def _start_move(self, angle: float, final_state: str, next_state: str):
        self.state, self.over_cnt = next_state, 0
        self.get_logger().info(f"Sætter → {final_state.upper()} ({angle}°)")
        self.servo.angle = angle

        def one_shot(tmr):
            self._pub_state(final_state)
            tmr.cancel()

        t = self.create_timer(self.MOVE_TIME_S, partial(one_shot, None))
        t.callback = partial(one_shot, t)

    # ─── strøm-watchdog ─────────────────────────────────────
    def cb_cur(self, msg: Float32):
        if self.state != "closing":
            return
        v = msg.data
        if v > self.MAX_VOLT:
            self.over_cnt += 1
            if self.over_cnt >= self.TRIP_SAMPLES:
                self.get_logger().error(f"‼︎ Overstrøm {v:.2f} V – stopper!")
                self.servo.angle = None
                self._pub_state("fault")
                self.state = "fault"
        else:
            self.over_cnt = 0

    # ─── status-helper ──────────────────────────────────────
    def _pub_state(self, word: str):
        self.state_pub.publish(String(data=word))
        if word == "opened":
            self.done_pub.publish(Empty())      # signal til indicator_node
            self.state = "idle"
        elif word == "closed":
            self.state = "idle"

    # ─── oprydning ─────────────────────────────────────────
    def destroy_node(self):
        self.servo.angle = None
        self.pca.deinit()
        super().destroy_node()

def main():
    rclpy.init()
    node = GrabNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
