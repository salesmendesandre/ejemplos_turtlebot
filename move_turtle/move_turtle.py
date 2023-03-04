import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def move(self):
        msg = Twist()
        msg.linear.x = 0.2  # Velocidad lineal en m/s
        msg.angular.z = 0.0  # Velocidad angular en rad/s
        self.publisher_.publish(msg)
        self.get_logger().info('Moviendo el TurtleBot en línea recta')

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = Turtlebot3Controller()
    turtlebot3_controller.move()
    rclpy.spin(turtlebot3_controller)
    turtlebot3_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()