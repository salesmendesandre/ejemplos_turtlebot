import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import fabs

class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('move_with_odomety')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)
        self.subscription_  # evitar que el objeto sea eliminado por el recolector de basura

        self.distance_to_move_ = 3.0  # distancia a recorrer en metros
        self.start_position_ = None  # posición inicial del robot

    def odometry_callback(self, msg):
        if self.start_position_ is None:
            # si es la primera vez que recibimos la odometría, establecemos la posición inicial
            self.start_position_ = msg.pose.pose.position
        else:
            # calculamos la distancia recorrida
            current_position = msg.pose.pose.position
            distance_traveled = fabs(current_position.x - self.start_position_.x)

            if distance_traveled >= self.distance_to_move_:
                # si hemos recorrido la distancia deseada, detenemos el robot y cerramos el nodo
                self.stop_moving()
                self.get_logger().info('Robot ha recorrido %f metros y se ha detenido', distance_traveled)
                rclpy.shutdown()

    def move(self):
        msg = Twist()
        msg.linear.x = 0.2  # Velocidad lineal en m/s
        msg.angular.z = 0.0  # Velocidad angular en rad/s
        self.publisher_.publish(msg)
        self.get_logger().info('Moviendo el TurtleBot en línea recta')

    def stop_moving(self):
        msg = Twist()
        msg.linear.x = 0.0  # Velocidad lineal en m/s
        msg.angular.z = 0.0  # Velocidad angular en rad/s
        self.publisher_.publish(msg)
        self.get_logger().info('Deteniendo el TurtleBot')

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = Turtlebot3Controller()
    turtlebot3_controller.move()
    rclpy.spin(turtlebot3_controller)
    turtlebot3_controller.destroy_node()

if __name__ == '__main__':
    main()