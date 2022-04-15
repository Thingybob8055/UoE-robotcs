import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import os

class MyRobot(Node):
    
    def __init__(self):
        super().__init__('random') 
        self.sub2 = self.create_subscription(Odometry, 'odom', self.odom_callback, 10) 
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #a list of variables declared that is used in this code
        self.my_x = 0 
        self.my_y = 0
        self.my_th = 0
        self.my_th_rad = 0
        self.goalcounter = 0
        self.get_flag = 0
        self.right_avoid_flag = 0
        self.left_avoid_flag= 0
        self.SAFEDIST = 0.3
        self.turn_flag = 0
        self.time_passed = 0.0

        self.file_name = "myfile.txt"
        self.get_file_name() #this function searches for duplicate file names and makes a new file with an incremented name
        self.myfile = open(self.file_name, "w+") #opens the file in write mode
        self.file_name = "myspeed.txt"
        self.get_file_name()
        self.myfile2 = open(self.file_name, "w+") #opens the file in write mode

    def get_file_name(self):
        if os.path.isfile(self.file_name): #if file is in path
            expand = 0
            while True:
                expand += 1
                new_file_name = self.file_name.split(".txt")[0] + str(expand) + ".txt" #new file name for example 'myfile1.txt'
                if os.path.isfile(new_file_name):
                    continue
                else:
                    self.file_name = new_file_name
                    break

    def odom_callback(self, msg): #subscribes to odom topic
        self.my_x = msg.pose.pose.position.x 
        self.my_y = msg.pose.pose.position.y

        # convert quaternian to Euler angles
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        self.my_th = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 * q2 + q3 * q3))) * 180 / math.pi
        self.my_th_rad = self.my_th*math.pi/180 #heading in radians
        #print ('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}'.format(self.my_x, self.my_y, self.my_th)) 
        
    def scan_callback(self, msg): #subscribes to laser scan topic
        #print(msg.ranges[90]) #left laser beam
        #print(msg.ranges[0]) #front laser beam
        #print(msg.ranges[270]) #right laser beam

        if( min(msg.ranges) <= self.SAFEDIST ): #if any of the distance is less that 0.3 in the laser scan array
            if((msg.ranges.index(min(msg.ranges)) >=0 and msg.ranges.index(min(msg.ranges)) <=70 ) ): #70 degree field of view
                self.right_avoid_flag = 1 #avoid right if obstacle is on left hand side
                print("[!]Left Detected")
            else:
                self.right_avoid_flag = 0
            if (msg.ranges.index(min(msg.ranges)) >=290 and msg.ranges.index(min(msg.ranges)) <=359):
                self.left_avoid_flag = 1
                print("[!]Right Detected")
            else:
                self.left_avoid_flag = 0
        else:
            self.left_avoid_flag = 0 #reset flgs
            self.right_avoid_flag = 0

    def move_distance(self, currentx, currenty): #takes the current position as argument
        move = Twist()
        move.linear.x = 0.2
        move.angular.z = 0.0
        self.pub.publish(move)
        self.myfile2.write(" %.3f, %.3f\n" % (move.linear.x, self.time_passed))
        distance = math.sqrt(math.pow(self.my_x - currentx, 2) + math.pow(self.my_y - currenty, 2)) #calculates distance moved
        if distance >= 0.5: #if distance is 0.5 or any set distance
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.turn_flag = 1  #makes the robot to start turning
            self.pub.publish(move)

    def turn(self, goal_angle): #takes argument the goal angle
        move = Twist()
        if goal_angle > math.pi:
            goal_angle = goal_angle - 2*math.pi #corrects the goal angle (only for one direction rotation)

        if goal_angle > -math.pi and goal_angle < 0:
            
            if abs(self.my_th_rad - goal_angle) < 0.2: #slows down the turning when reaching the goal heading
                move.linear.x = 0.0
                move.angular.z = 0.2
                print("[!]TURNING<<<")
                self.pub.publish(move)
            else:
                move.linear.x = 0.0
                move.angular.z = 0.5
                print("[!]TURNING<<<")
                self.pub.publish(move)
            if abs(self.my_th_rad - goal_angle) < 0.03: #turns till the angle difference to the goal is small
                print("[!]TURNING DONE<<<")
                move.linear.x = 0.0
                move.angular.z = 0.0  
                self.pub.publish(move)
                self.get_flag = 0
                self.turn_flag = 0
        
        print("[!]GOAL ANGLE IS: %f", goal_angle*(180/math.pi))
        if goal_angle >= 0 and goal_angle <= 180:
            
            if abs(self.my_th_rad - goal_angle) < 0.2:
                move.linear.x = 0.0
                move.angular.z = 0.2
                self.pub.publish(move)
            else:
                move.linear.x = 0.0
                move.angular.z = 0.5
                print("[!]TURNING<<<")
                self.pub.publish(move)
            if abs(self.my_th_rad - goal_angle) < 0.03:
                print("[!]TURNING DONE")
                move.linear.x = 0.0
                move.angular.z = 0.0  
                self.pub.publish(move)
                self.get_flag = 0
                self.turn_flag = 0

    def timer_callback(self):
        move = Twist()
        self.time_passed = self.time_passed + 0.1 #calculates the time passed for data collection purposes
        self.myfile.write(" %.3f, %.3f, %.3f\n" % (self.my_x, self.my_y, self.time_passed)) #x, y, time passed

        if self.right_avoid_flag == 1: # obstacle avoidance if obstacle is detected
            #move.linear.x = 0.2
            move.angular.z = -1.5
            self.myfile2.write(" %.3f, %.3f\n" % (move.linear.x, self.time_passed)) #speed, time passed
            self.pub.publish(move)
            self.get_flag = 0
            self.turn_flag = 0
        elif self.left_avoid_flag == 1:
            #move.linear.x = 0.2
            move.angular.z = 1.5
            self.myfile2.write(" %.3f, %.3f\n" % (move.linear.x, self.time_passed)) #speed, time passed
            self.pub.publish(move)
            self.get_flag = 0
            self.turn_flag = 0
        else:
            if self.get_flag == 0: #gets the current co-ordinate details
                self.get_flag = 1
                self.currentx = self.my_x
                self.currenty = self.my_y
                self.currentth = self.my_th_rad
                print("[!]GOT CURRENT DETAILS: x =  %f y =  %f yaw = %f" %(self.currentx, self.currenty, self.currentth))
            if self.turn_flag == 0:
                self.move_distance(self.currentx, self.currenty) #move short distance
            else:
                self.turn(self.currentth+(math.pi/6)) #turn fixed angle

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