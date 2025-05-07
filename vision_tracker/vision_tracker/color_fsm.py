import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ColorFSM(Node):
    def __init__(self):
        super().__init__('color_fsm')
        self.subscriber = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        self.last_color = None
        self.wait_until = None

        # Parámetros de movimiento
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.align_gain = 0.5 # Ganancia proporcional para desacelerar el giro

        self.color_ranges = {
            'green':   ((35, 50, 50),  (85, 255, 255)),
            'orange':  ((10, 100, 100), (25, 255, 255)),
            'blue':    ((100, 100, 100), (130, 255, 255)),
            'yellow':  ((25, 100, 100), (35, 255, 255)),
        }

    def detect_color(self, hsv, color_name):
        lower, upper = self.color_ranges[color_name]
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 30:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    return True, area, cx
        return False, 0, None

    def image_callback(self, msg):
        try:
            if self.wait_until and time.time() < self.wait_until:
                return

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            twist = Twist()

            detected, area, cx = self.detect_color(hsv, 'green')
            if detected:
                width = cv_image.shape[1]
                error_x = (cx - width // 2) / (width // 2)  # Normalizado [-1, 1]

                if self.last_color in ['orange', 'blue']:
                    # Venimos de girar: seguimos girando suavemente hasta alinear
                    twist.angular.z = -self.align_gain * error_x
                    twist.linear.x = 0.0
                    self.get_logger().info(f"🎯 Transición a verde: alineando (error_x={error_x:.2f})")
                    if abs(error_x) < 0.1:
                        self.last_color = 'green'  # Considerar ya alineado
                else:
                    # Ya alineado o veníamos de otro estado
                    if area >= 300:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.get_logger().info(f"🛑 Verde muy cerca (área={area:.0f}): deteniéndose")
                    elif abs(error_x) < 0.2:
                        twist.linear.x = self.linear_speed
                        twist.angular.z = -self.align_gain * error_x
                        self.get_logger().info(f"🟢 Verde centrado: avanzando")
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = -self.align_gain * error_x
                        self.get_logger().info(f"🟢 Verde mal alineado (error_x={error_x:.2f}): ajustando")

                # Siempre actualizar
                self.last_color = 'green'

            elif self.detect_color(hsv, 'orange')[0]:
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
                self.last_color = 'orange'
                self.get_logger().info("🟠 Naranja detectada: girando a la derecha")

            elif self.detect_color(hsv, 'blue')[0]:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
                self.last_color = 'blue'
                self.get_logger().info("🔵 Azul detectada: girando a la izquierda")

            elif self.detect_color(hsv, 'yellow')[0]:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.wait_until = time.time() + 5
                self.last_color = 'yellow'
                self.get_logger().info("🟡 Amarilla detectada: pausa de 5 segundos")

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warn("⚫ No se detectó ninguna esfera: deteniéndose")
                self.last_color = 'none'

            self.publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
