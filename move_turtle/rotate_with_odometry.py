import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Vector3
from nav_msgs.msg import Odometry
import math


class RotateBot(Node):

    def __init__(self):
        super().__init__('rotate_bot')

        # Suscribirse a la odometría para obtener la orientación actual del robot
        self.subscription_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Publicar mensajes de velocidad angular
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Valores de error de orientación y velocidad angular máxima
        self.target_yaw_ = 1.5708  # 90 grados en radianes
        self.yaw_error_ = 0.05  # 2.8 grados en radianes
        self.max_angular_vel_ = 0.2  # radianes/segundo

    def odom_callback(self, msg):
        # Extraer la orientación actual del robot a partir de la odometría
        current_yaw = math.atan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
                                 1.0 - 2.0 * (msg.pose.pose.orientation.z * msg.pose.pose.orientation.z))

        # Calcular la diferencia de orientación necesaria
        yaw_diff = self.target_yaw_ - current_yaw

        # Calcular la velocidad angular proporcional al error de orientación
        angular_vel = min(self.max_angular_vel_, max(-self.max_angular_vel_, 1.5 * yaw_diff))

        # Crear un mensaje Twist y publicarlo
        twist_msg = Twist()
        twist_msg.angular.z = angular_vel
        self.publisher_.publish(twist_msg)

        # Si la diferencia de orientación es menor que el error de orientación, detener el movimiento
        if abs(yaw_diff) <= self.yaw_error_:
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            self.get_logger().info('Bot girado 90 grados')
            #self.subscription_.dispose()
            #self.destroy_node()
            #rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    node = RotateBot()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
