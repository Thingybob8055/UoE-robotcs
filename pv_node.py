import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('publish_velocity_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel',10)
        timer_period = 0.2 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        move = Twist()
        self.i = self.i + 1
        #move.linear.x = 0.2
        if self.i == 3:
            move.angular.z = 0.0
            self.pub.publish(move)
            rclpy.shutdown()
        move.angular.z = 0.3
        print("linear X: ", move.linear.x)
        print("Angular Z: ", move.angular.z)
        self.pub.publish(move)

def main(args=None):
    rclpy.init(args=args)
    publish_velocity_node = MinimalPublisher()
    rclpy.spin(publish_velocity_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publish_velocity_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
