#!/usr/bin/env python3
"""
linear_actuator_node – v4
------------------------------------------------
Sekvens:
1.  /signal            → kør servo NED
2.  /grab_cmd:"close"  → griber lukker
3.  5 s hold           → kør servo OP
4a. Vent på /limit_state:"up"
    - eller 2 s timeout
5.  20 s hold          → /grab_cmd:"open"
6.  /grab_state:"opened" → tilbage til IDLE
"""

import time, enum, board, busio, rclpy
from rclpy.node   import Node
from std_msgs.msg import Empty, String
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo


class State(enum.Enum):
    IDLE             = enum.auto()
    MOVING_DOWN      = enum.auto()
    WAIT_CLOSED      = enum.auto()
    HOLD_CLOSED      = enum.auto()
    MOVING_UP        = enum.auto()
    WAIT_UP_SWITCH   = enum.auto()
    WAIT_BEFORE_OPEN = enum.auto()
    WAIT_OPENED      = enum.auto()


# ─── konstanter (tilpas til mekanik) ───────────────────────────
PCA_CHANNEL        = 0
ANGLE_DOWN         = 165.0
ANGLE_UP           = 25.0

DOWN_TIME          = 0.8        # sek. ned-bevægelse
HOLD_CLOSED_TIME   = 5.0        # sek. griber holdes lukket
UP_TIMEOUT         = 2.0        # sek. at vente på end-switch
WAIT_BEFORE_OPEN   = 20.0       # sek. pause før open-grib
# ───────────────────────────────────────────────────────────────


class LinearActuator(Node):
    def __init__(self):
        super().__init__("linear_actuator_node")

        # hardware
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.servo = Servo(self.pca.channels[PCA_CHANNEL])

        # pub/sub
        self.grab_pub = self.create_publisher(String, "/grab_cmd", 10)
        self.create_subscription(Empty,  "/signal",      self.cb_sig, 10)
        self.create_subscription(String, "/grab_state",  self.cb_gst, 10)
        self.create_subscription(String, "/limit_state", self.cb_lmt, 10)

        # state-maskine
        self.state    = State.IDLE
        self.deadline = 0.0
        self.timer    = self.create_timer(0.1, self.loop)   # 10 Hz

        self.get_logger().info("⏳ Klar – venter på /signal.")

    # ────── callbacks ──────────────────────────────────────────
    def cb_sig(self, _: Empty):
        if self.state != State.IDLE:
            self.get_logger().warn("Ignorerer /signal – ikke i IDLE.")
            return
        self.get_logger().info("🚀 /signal modtaget → kører NED.")
        self.servo.angle = ANGLE_DOWN
        self.state    = State.MOVING_DOWN
        self.deadline = time.time() + DOWN_TIME

    def cb_gst(self, msg: String):
        if msg.data == "closed" and self.state == State.WAIT_CLOSED:
            self.get_logger().info("✅ Griber lukket – starter 5 s hold.")
            self.state    = State.HOLD_CLOSED
            self.deadline = time.time() + HOLD_CLOSED_TIME

        elif msg.data == "opened" and self.state == State.WAIT_OPENED:
            self.get_logger().info("✅ Sekvens færdig – tilbage i IDLE.")
            self.state = State.IDLE
            self.servo.angle = None

        elif msg.data == "fault":
            self.get_logger().error("‼︎ Griber meldte fault – afbryder cyklus.")
            self.state = State.IDLE
            self.servo.angle = None

    def cb_lmt(self, msg: String):
        if msg.data == "up" and self.state == State.WAIT_UP_SWITCH:
            self.get_logger().info("🟢 End-switch aktiv – 20 s pause.")
            self.state    = State.WAIT_BEFORE_OPEN
            self.deadline = time.time() + WAIT_BEFORE_OPEN

    # ────── hoved-loop ─────────────────────────────────────────
    def loop(self):
        now = time.time()

        if self.state == State.MOVING_DOWN and now >= self.deadline:
            self.get_logger().info("Nede – sender /grab_cmd:'close'.")
            self.grab_pub.publish(String(data="close"))
            self.state = State.WAIT_CLOSED

        elif self.state == State.HOLD_CLOSED and now >= self.deadline:
            self.get_logger().info("⏫ Kører OP.")
            self.servo.angle = ANGLE_UP
            self.state    = State.WAIT_UP_SWITCH
            self.deadline = now + UP_TIMEOUT          # start timeout-ur

        elif self.state == State.WAIT_UP_SWITCH and now >= self.deadline:
            # timeout trigges før /limit_state:"up"
            self.get_logger().warn("⚠️  End-switch timeout – fortsætter alligevel.")
            self.state    = State.WAIT_BEFORE_OPEN
            self.deadline = now + WAIT_BEFORE_OPEN

        elif self.state == State.WAIT_BEFORE_OPEN and now >= self.deadline:
            self.get_logger().info("🟢 Sender /grab_cmd:'open'.")
            self.grab_pub.publish(String(data="open"))
            self.state = State.WAIT_OPENED

    # ────── oprydning ──────────────────────────────────────────
    def destroy_node(self):
        self.servo.angle = None
        self.pca.deinit()
        super().destroy_node()


def main():
    rclpy.init()
    node = LinearActuator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
