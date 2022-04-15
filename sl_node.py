import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

kp_distance = 1  
ki_distance = 0.001
kd_distance = 0.05

kp_angle = 0.95 
ki_angle = 0.03
kd_angle = 0.05


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('subscribe_laser_node')
        self.sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.sub # prevent unused variable warning
        self.pub = self.create_publisher(Twist, 'cmd_vel',10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.front = 0
        self.right = 0
        self.left = 0

    def pid_controller(self, goal_x, goal_y):
        move = Twist()
        turn_angle = math.atan2(goal_y - self.my_y, goal_x- self.my_x)

        if turn_angle <= -math.pi/4 or turn_angle > math.pi/4: 
            if goal_y < 0 and self.my_y < goal_y:
                turn_angle = -2*math.pi + turn_angle 
            elif goal_y >=0 and self.my_y > goal_y:
                turn_angle = 2*math.pi + turn_angle
        if self.last_rotation > math.pi-0.1 and self.my_th_rad <=0:
            self.my_th_rad = 2*math.pi + self.my_th_rad
        elif self.last_rotation < -math.pi+0.1 and self.my_th_rad > 0:
            self.my_th_rad = -2*math.pi + self.my_th_rad

        distance = math.sqrt(math.pow(goal_x - self.my_x, 2) + math.pow(goal_y - self.my_y, 2)) 
        if distance < 0.03:
            #print("[!]Stopped")
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.pub.publish(move)
            rclpy.shutdown()

        diff_angle = turn_angle - self.previous_angle
        diff_distance = distance - self.previous_distance

        control_signal_angle = kp_angle*turn_angle + ki_angle*self.total_angle + kd_angle*diff_angle
        control_signal_distance = kp_distance*distance + ki_distance*self.total_distance + kd_distance*diff_distance

        move.angular.z = (control_signal_angle) - self.my_th_rad
        move.linear.x = min(control_signal_distance, 0.2)

        if move.angular.z > 0:
            move.angular.z = min(move.angular.z, 0.5)
        else:
            move.angular.z = max(move.angular.z, -0.5)

        self.last_rotation = self.my_th_rad

        self.pub.publish(move)

        diff_angle = turn_angle - self.previous_angle
        diff_distance = distance - self.previous_distance
    
    def timer_callback(self):
        move = Twist()
        # move.linear.x = 0.2

        if self.front <=0.5:
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.pub.publish(move)
        else:
            self.pid_controller(1.5, 3.2)


        # move.angular.z = 0.1
    
    def scan_callback(self, msg):
        print(msg.ranges[90]) #left laser beam
        print(msg.ranges[0]) #front laser beam
        print(msg.ranges[270]) #right laser beam
        self.front = msg.ranges[0]
        self.right = msg.ranges[270]
        self.left = msg.ranges[90]

def main(args=None):
    rclpy.init(args=args)
    subscribe_laser_node = MinimalSubscriber()
    rclpy.spin(subscribe_laser_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscribe_laser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
