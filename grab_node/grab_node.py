#!/usr/bin/env python3
# grab_node.py
# ------------------------------------------------------------------------
# - Subscriber: /grab_cmd      ("close" | "open")
# - Publisher : /grab_state    ("closed" | "opened" | "fault")
# - Publisher : /sequence_done (Empty)  - sendes når griberen er åbnet igen
# - Overstrøms‑watchdog på /motor_current (volt fra CMS)
# ------------------------------------------------------------------------

# ─── Imports ─────────────────────────────────────────────────────────────
import time, rclpy, board, busio
from rclpy.node import Node
from std_msgs.msg import String, Float32, Empty
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# ─── Konstanter ──────────────────────────────────────────────────────────
PCA_CHANNEL   = 1
SERVO_FREQ    = 50
ANGLE_OPEN    = 20
ANGLE_CLOSE   = 158
MOVE_TIME_S   = 1.0          # tid før status‑publish

VOLT_PER_AMP  = 3.83         # 0,22 Ω × 17,41
MAX_CURRENT   = 0.80         # A
MAX_VOLT      = MAX_CURRENT * VOLT_PER_AMP   # ≈ 3,06 V
TRIP_SAMPLES  = 5
CHECK_PERIOD  = 0.05         # hovedløkkens periode (sek.)

STATE_IDLE    = "idle"
STATE_CLOSING = "closing"
STATE_OPENING = "opening"
STATE_FAULT   = "fault"

# ─────────────────────────────────────────────────────────────────────────
class GrabNode(Node):
    def __init__(self):
        super().__init__("grab_node")

        # hardware‑setup
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = SERVO_FREQ
        self.servo = servo.Servo(self.pca.channels[PCA_CHANNEL],
                                 min_pulse=500, max_pulse=2700)

        # pub/sub
        self.state_pub = self.create_publisher(String, "/grab_state", 10)
        self.done_pub  = self.create_publisher(Empty,  "/sequence_done", 10)

        self.create_subscription(String,  "/grab_cmd",      self.cb_cmd, 10)
        self.create_subscription(Float32, "/motor_current", self.cb_cur, 10)

        # interne variabler
        self.state         = STATE_IDLE
        self.over_cnt      = 0
        self.move_deadline = 0.0
        self.final_state   = None   # "opened"/"closed"

        # hovedløkke
        self.create_timer(CHECK_PERIOD, self.loop)

        self.get_logger().info("Grab‑node klar – venter på /grab_cmd.")

    # ─── Kommando‑callback ──────────────────────────────────────────────
    def cb_cmd(self, msg: String):
        cmd = msg.data.lower().strip()
        if cmd == "close" and self.state == STATE_IDLE:
            self._start_move(ANGLE_CLOSE, "closed", STATE_CLOSING)
        elif cmd == "open" and self.state == STATE_IDLE:
            self._start_move(ANGLE_OPEN,  "opened", STATE_OPENING)
        else:
            self.get_logger().warn(f"Ignorerer /grab_cmd: {cmd} (state={self.state})")

    def _start_move(self, angle: float, final_state: str, next_state: str):
        self.get_logger().info(f"Servo → {angle}°  ({final_state.upper()})")
        self.servo.angle  = angle
        self.state        = next_state
        self.over_cnt     = 0
        self.final_state  = final_state
        self.move_deadline = time.time() + MOVE_TIME_S

    # ─── Strøm‑watchdog ─────────────────────────────────────────────────
    def cb_cur(self, msg: Float32):
        if self.state != STATE_CLOSING:
            return
        v = msg.data
        if v > MAX_VOLT:
            self.over_cnt += 1
            if self.over_cnt >= TRIP_SAMPLES:
                self.get_logger().error(f"‼︎ Overstrøm {v:.2f} V – stopper!")
                self.servo.angle = None
                self._publish_state(STATE_FAULT)
        else:
            self.over_cnt = 0

    # ─── Hovedløkke ─────────────────────────────────────────────────────
    def loop(self):
        # afslut bevægelse efter MOVE_TIME_S
        if self.move_deadline and time.time() >= self.move_deadline:
            self.move_deadline = 0.0
            if self.final_state:
                self._publish_state(self.final_state)
                self.final_state = None

    # ─── Status‑publish helper ──────────────────────────────────────────
    def _publish_state(self, word: str):
        self.state_pub.publish(String(data=word))
        if word == "opened":
            self.done_pub.publish(Empty())
            self.state = STATE_IDLE
        elif word == "closed":
            self.state = STATE_IDLE
        elif word == STATE_FAULT:
            self.state = STATE_FAULT

    # ─── Oprydning ──────────────────────────────────────────────────────
    def destroy_node(self):
        self.servo.angle = None
        self.pca.deinit()
        super().destroy_node()

# ─── Main‑indgang ───────────────────────────────────────────────────────

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
