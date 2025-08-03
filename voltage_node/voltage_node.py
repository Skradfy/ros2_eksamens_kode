#!/usr/bin/env python3

# voltage_node.py
# ──────────────────────────────────────────────────────────────────
# - Måler motorstrøm via high-side shunt + MCP3008 (10 bit)
# - Udgiver strømmen som Ampere på topic /motor_current
# ──────────────────────────────────────────────────────────────────

# ─── Imports ──────────────────────────────────────────────────────
import spidev                       # SPI-bibliotek til Pi5
import rclpy                        # ROS2 Python-client
from rclpy.node import Node
from std_msgs.msg import Float32    # Float message

# ─── Konstanter ───────────────────────────────────────────────────
V_REF       = 3.3    # ADC reference-spænding (Volt)
GAIN_V_PER_A = 3.83  # Forstærker giver 3.83 V pr. Ampere
ADC_CHANNEL = 0      # MCP3008 kanal 0
SAMPLE_RATE = 50     # Hz – hvor ofte vi måler og publicerer

# ──────────────────────────────────────────────────────────────────
class VoltageNode(Node):
    """ROS2-node der konverterer shunt-spænding til ampere."""

    def __init__(self):
        super().__init__("voltage_node")

        # ── SPI: åbner /dev/spidev0.0 ─────────────────────────────
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)              # bus 0, chip-select 0
        self.spi.max_speed_hz = 1_000_000

        # ── ROS-publisher ─────────────────────────────────────────
        self.publisher = self.create_publisher(Float32,
                                               "/motor_current", 10)

        # ── Timer kalder self.measure()
        self.create_timer(1.0 / SAMPLE_RATE, self.measure)
        self.get_logger().info(f"VoltageNode kører {SAMPLE_RATE} Hz")

    # ──────────────────────────────────────────────────────────────
    # 1. Læs rå-ADC-tal (0-1023) fra MCP3008
    # ──────────────────────────────────────────────────────────────
    def _read_adc_raw(self) -> int:
        # SPI-ramme: [start-bit, kanal-vælg, dummy]
        frame = [1, (8 + ADC_CHANNEL) << 4, 0]
        resp  = self.spi.xfer2(frame)
        raw   = ((resp[1] & 0x03) << 8) | resp[2]  # 10-bit tal
        return raw

    # ──────────────────────────────────────────────────────────────
    # 2. Mål → konverter → publicer
    # ──────────────────────────────────────────────────────────────
    def measure(self):
        raw    = self._read_adc_raw()           # 0-1023
        volts  = raw * V_REF / 1023.0           # ADC → Volt
        amps   = volts / GAIN_V_PER_A           # Volt → Ampere
        self.publisher.publish(Float32(data=amps))


# ─── Main-indgang ─────────────────────────────────────────────────
def main():
    rclpy.init()
    node = VoltageNode()
    try:
        rclpy.spin(node)        # kør, indtil Ctrl-C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
