from ftplib import parse150
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05
final_angle = math.pi/2
final = 90

final_posx = 0.0
final_posy = 1.0

kp_distance = 1
ki_distance = 0.001
kd_distance = 0.05

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('subscribe_odometry_node')
        self.sub = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.sub # prevent unused variable warning
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.previous_angle = 0
        self.total_angle = 0
        self.get_goal_distance()

    def get_goal_distance(self):
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_distance = math.sqrt(math.pow(final_posx - self.current_x, 2) + math.pow(final_posy - self.current_y, 2))
        self.distance = self.goal_distance
        self.previous_distance = 0
        self.total_distance = 0

    def turn(self):
        x = 2
        move = Twist()
        move.linear.x = 0.0
        diff_angle = final_angle - self.previous_angle
        print("[!]Running")
        control_signal_angle = kp_angle*final_angle + ki_angle*self.total_angle + kd_angle*diff_angle
        move.angular.z = (control_signal_angle) - self.my_th_rad

        if move.angular.z > 0:
            move.angular.z = min(move.angular.z, 1.5)
            move.angular.z
        else:
            move.angular.z = max(move.angular.z, -1.5)
            move.angular.z

        self.pub.publish(move)      

    def move(self):
        move = Twist()
        if self.distance < 0.05:
            print("[!]Stopped")
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.pub.publish(move)
            rclpy.shutdown()
        diff_distance = self.distance - self.previous_distance

        self.distance = math.sqrt(math.pow(final_posx - self.my_x, 2) + math.pow(final_posy - self.my_y, 2))

        control_signal_distance = kp_distance*self.distance + ki_distance*self.total_distance + kd_distance*diff_distance

        # your controller code should be replace the following two lines.
        move.linear.x = min(control_signal_distance, 0.2)
        move.angular.z = 0.0
        self.previous_distance = self.distance
        self.total_distance = self.total_distance + self.distance
        print("[!]Running %f" % self.distance)
        #myfile.write(" x = %.3f y =  %.3f theta =  %.3f\n " % (self.my_x, self.my_y, self.my_th))
        self.pub.publish(move)

    def timer_callback(self):
        move = Twist()
        
        self.turn()
        


    
        # move.angular.z = min(0.5*(final - self.my_th), 1.5)

        # self.pub.publish(move)
        # if self.my_th >=90:
        #     move.linear.x = 0.2
        #     move.angular.z = 0.0
        #     self.pub.publish(move)
    
    def odom_callback(self, msg):
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y

        # convert quaternian to Euler angles
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        self.my_th = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 * q2 + q3 * q3))) * 180 / math.pi
        self.my_th_rad = self.my_th*math.pi/180
        print ('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}'.format(self.my_x, self.my_y, self.my_th))

def main(args=None):
        rclpy.init(args=args)
        subscribe_odometry_node = MinimalSubscriber()
        rclpy.spin(subscribe_odometry_node)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        subscribe_odometry_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
