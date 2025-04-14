import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller_node')
        self.subscriber = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # Parámetros de control
        self.v_max = 0.5
        self.k_p = 1.5
        self.k_dist = 0.00012  # Ganancia para control proporcional de distancia
        self.alpha = 0.6       # Filtro EMA para error horizontal

        self.last_error = 0.0
        self.last_known_direction = 1
        self.lost_counter = 0
        self.max_lost = 10

        self.target_area = 6000  # Área ideal para mantener distancia
        self.area_buffer = []    # Para suavizar lectura de área

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Rango de color rojo
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            twist = Twist()

            if contours:
                self.lost_counter = 0
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)

                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    width = cv_image.shape[1]
                    raw_error = (cx - width / 2) / (width / 2)  # Normalizado [-1, 1]
                    error_x = self.alpha * raw_error + (1 - self.alpha) * self.last_error
                    self.last_error = error_x

                    # Corregido: usar la última dirección bruta (no filtrada)
                    if abs(raw_error) > 0.02:
                        self.last_known_direction = -1 if raw_error < 0 else 1

                    # Área suavizada
                    area = cv2.contourArea(c)
                    self.area_buffer.append(area)
                    if len(self.area_buffer) > 5:
                        self.area_buffer.pop(0)
                    area_smoothed = sum(self.area_buffer) / len(self.area_buffer)

                    # Control proporcional de distancia
                    error_area = self.target_area - area_smoothed
                    linear = self.k_dist * error_area
                    linear = max(-0.3, min(self.v_max, linear))

                    # Control angular
                    angular = -self.k_p * error_x

                    twist.linear.x = linear
                    twist.angular.z = angular

                    self.get_logger().info(
                        f"error_x={error_x:.2f}, area={area_smoothed:.0f}, v={linear:.2f}, w={angular:.2f}"
                    )

            else:
                # Objeto perdido
                self.lost_counter += 1

                if self.lost_counter >= self.max_lost:
                    twist.linear.x = 0.0
                    twist.angular.z = 1.5 * self.last_known_direction
                    self.get_logger().warn("🔍 Esfera perdida. Girando para buscarla...")
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

            self.publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
