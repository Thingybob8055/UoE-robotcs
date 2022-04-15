import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import os

kp_distance = 1  
ki_distance = 0.001
kd_distance = 0.05

kp_angle = 0.95 
ki_angle = 0.03
kd_angle = 0.05

class MyRobot(Node):
    
    def __init__(self):
        super().__init__('test_node') 
        self.sub2 = self.create_subscription(Odometry, 'odom', self.odom_callback, 10) 
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.my_x = 0 
        self.my_y = 0
        self.my_th = 0
        self.previous_angle = 0
        self.total_angle = 0
        self.goalcounter = 0
        self.last_rotation = 0
        self.previous_distance = 0
        self.total_distance = 0
        self.front = 0
        self.right = 0
        self.left = 0
        self.get_flag = 0
        self.avoid_flag = 0
        self.right_avoid_flag = 0
        self.left_avoid_flag= 0
        self.SAFEDIST = 0.3
        self.time_passed = 0.0

        self.file_name = "myfile.txt"
        self.get_file_name()
        self.myfile = open(self.file_name, "w+") #opens the file in write mode

    def get_file_name(self):
        if os.path.isfile(self.file_name):
            expand = 1
            while True:
                expand += 1
                new_file_name = self.file_name.split(".txt")[0] + str(expand) + ".txt"
                if os.path.isfile(new_file_name):
                    continue
                else:
                    self.file_name = new_file_name
                    break

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
        #print ('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}'.format(self.my_x, self.my_y, self.my_th)) 

    def scan_callback(self, msg):
        #print(msg.ranges[90]) #left laser beam
        #print(msg.ranges[0]) #front laser beam
        #print(msg.ranges[270]) #right laser beam
        self.front = msg.ranges[0]
        self.right = msg.ranges[270]
        self.left = msg.ranges[90]

        if( min(msg.ranges) <= self.SAFEDIST ):
            if((msg.ranges.index(min(msg.ranges)) >=0 and msg.ranges.index(min(msg.ranges)) <=50 ) ):
                self.right_avoid_flag = 1
                print("[!]Left Detected")
            else:
                self.right_avoid_flag = 0
            if (msg.ranges.index(min(msg.ranges)) >=300 and msg.ranges.index(min(msg.ranges)) <=359):
                self.left_avoid_flag = 1
                print("[!]Right Detected")
            else:
                self.left_avoid_flag = 0
        else:
            self.left_avoid_flag = 0
            self.right_avoid_flag = 0
        
        # if( min(msg.ranges) <=0.3 ):
        #     if((msg.ranges.index(min(msg.ranges)) >=0 and msg.ranges.index(min(msg.ranges)) <=30 ) or (msg.ranges.index(min(msg.ranges)) >=320 and msg.ranges.index(min(msg.ranges)) <=359)):
        #         self.avoid_flag = 1
        #     else:
        #         self.avoid_flag = 0
        # else:
        #     self.avoid_flag = 0

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
            self.goalcounter = self.goalcounter + 1
            self.total_distance = 0
            self.previous_distance = 0

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
        self.time_passed = self.time_passed + 0.1
        self.myfile.write(" %.3f, %.3f, %.3f\n" % (self.my_x, self.my_y, self.time_passed)) #x, y, time passed

        if self.right_avoid_flag == 1:
            #move.linear.x = 0.2
            move.angular.z = -1.5
            self.pub.publish(move)
        elif self.left_avoid_flag == 1:
            #move.linear.x = 0.2
            move.angular.z = 1.5
            self.pub.publish(move)
        # if self.avoid_flag == 1:
        #     #move.linear.x = 0.2
        #     move.angular.z = 1.5
        #     self.pub.publish(move)
        else:
            ############ UNCOMMENT THESE FOR CUSTOM WORLD ##############
            # if self.goalcounter == 0:
            #     self.pid_controller(3.5, 0.5)
            # if self.goalcounter == 1:
            #     self.pid_controller(3.5, 3.5)
            # if self.goalcounter == 2:
            #     self.pid_controller(-2.0, 4.5)
            # if self.goalcounter == 3:
            #     self.pid_controller(-1.5, -2.0)
            # if self.goalcounter == 4:
            #     self.pid_controller(1.5, -3.0)
            # if self.goalcounter == 5:
            #     self.pid_controller(0.0, 0.0)
            # if self.goalcounter == 6:
            #     move.linear.x = 0.0
            #     move.angular.z = 0.0
            #     self.myfile.close()
            #     self.pub.publish(move)
            #     rclpy.shutdown()

            ############ UNCOMMENT THESE FOR TURTLEBOT3 WORLD ##############
            if self.goalcounter == 0:
                self.pid_controller(1.5, 1.5)
            if self.goalcounter == 1:
                self.pid_controller(0.5, -1.5)
            if self.goalcounter == 2:
                self.pid_controller(-2.0, -0.5)
            if self.goalcounter == 3:
                move.linear.x = 0.0
                move.angular.z = 0.0
                self.myfile.close()
                self.pub.publish(move)
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    myrobot_node = MyRobot()
    rclpy.spin(myrobot_node)
    #myrobot_node.Moving()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    myrobot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()