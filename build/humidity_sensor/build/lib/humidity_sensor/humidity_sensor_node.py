#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import board
import adafruit_dht

class HumiditySensorNode(Node):
    def __init__(self):
        super().__init__('humidity_sensor_node')

        # GPIO4 = pin 7
        self.dht_device = adafruit_dht.DHT11(board.D4)

        # temp and humidity publishers
        self.temp_pub = self.create_publisher(Float32, '/temperature_c', 10)
        self.hum_pub = self.create_publisher(Float32, '/humidity_percent', 10)

        # Read every 2 seconds
        self.timer = self.create_timer(3.0, self.read_sensor)

        self.get_logger().info('DHT11 node started. Reading temperature and humidity...')

    def read_sensor(self):
        try:
            temperature_c = self.dht_device.temperature
            humidity = self.dht_device.humidity

            if temperature_c is None or humidity is None:
                self.get_logger().warn('Sensor returned None reading.')
                return

            temp_msg = Float32()
            hum_msg = Float32()

            temp_msg.data = float(temperature_c)
            hum_msg.data = float(humidity)

            self.temp_pub.publish(temp_msg)
            self.hum_pub.publish(hum_msg)

            self.get_logger().info(
                f'Temperature: {temperature_c:.1f} C | Humidity: {humidity:.1f} %'
            )
            
        except RuntimeError as e:
            # DHT sensors commonly throw occasional transient read errors
            self.get_logger().warn(f'Read retry: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    def destroy(self):
        try:
            self.dht_device.exit()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HumiditySensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
                