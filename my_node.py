import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
myfile = open("myodom.txt", "w+") #opens the file in write mode

kp_distance = 1  #PID constants for distance (linear movement)
ki_distance = 0.001
kd_distance = 0.05

kp_angle = 0.95 #PID movement for angular movement
ki_angle = 0.03
kd_angle = 0.05

initial_heading = 0.00 #variables for each of the goal headings when going in a square trajectory
final_angle = math.pi/2
final_angle2 = math.pi
final_angle3 = -math.pi/2
class MyRobot(Node):
    
    def __init__(self):
        super().__init__('myrobot_node')
        self.get_initial_information() #get initial robot informatiob
        self.sub2 = self.create_subscription(Odometry, 'odom', self.odom_callback, 10) #subscribe to odom topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) #publish velocity topic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.my_x = 0 #initialising variables
        self.my_y = 0
        self.my_th = 0
        self.previous_angle = 0
        self.total_angle = 0
        self.get_flag = 0

    def get_initial_information(self):
        self.goalcounter = 0 #initialisng variables regardling initial information
        self.last_rotation = 0
        self.previous_distance = 0
        self.total_distance = 0


    def odom_callback(self, msg): #this function gets odometry data, prints them to screen and text fule
        self.my_x = msg.pose.pose.position.x #x and y co-ordinates
        self.my_y = msg.pose.pose.position.y

        # convert quaternian to Euler angles
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        self.my_th = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 * q2 + q3 * q3))) * 180 / math.pi
        self.my_th_rad = self.my_th*math.pi/180 #theta in radians
        print ('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}'.format(self.my_x, self.my_y, self.my_th)) #print to terminal and text file
        myfile.write("x = %.3f y =  %.3f theta =  %.3f\n" % (self.my_x, self.my_y, self.my_th))

    def pid_turn_controller(self, turn_angle, goal_y): ##PID controller for angular movement
        move = Twist()
        move.linear.x = 0.0

        if turn_angle <= -math.pi/4 or turn_angle > math.pi/4: #if the agle is in this range
            if goal_y < 0 and self.my_y < goal_y:
                turn_angle = -2*math.pi + turn_angle #turn angle (based on the goal) is updated
            elif goal_y >=0 and self.my_y > goal_y:
                turn_angle = 2*math.pi + turn_angle
        if self.last_rotation > math.pi-0.1 and self.my_th_rad <=0: #edits the theta based on the previous rotation
            self.my_th_rad = 2*math.pi + self.my_th_rad
        elif self.last_rotation < -math.pi+0.1 and self.my_th_rad > 0:
            self.my_th_rad = -2*math.pi + self.my_th_rad

        diff_angle = turn_angle - self.previous_angle #calculates the derivative angle error
        print("[!]Running")
        control_signal_angle = kp_angle*turn_angle + ki_angle*self.total_angle + kd_angle*diff_angle #the pid control signal is caluculated
        move.angular.z = (control_signal_angle) - self.my_th_rad #the control signal is applied to the rotation speed

        if move.angular.z > 0:
            move.angular.z = min(move.angular.z, 0.5)
        else:
            move.angular.z = max(move.angular.z, -0.5)
        self.pub.publish(move)   #publush
        self.last_rotation = self.my_th_rad #store current angle into previous angle variable   

    def pid_move_controller(self, finalx, finaly): #pid controller for linear movement
        move = Twist()

        distance = math.sqrt(math.pow(finalx - self.my_x, 2) + math.pow(finaly - self.my_y, 2)) #calculates the current error
        if distance < 0.03: #increments goal counter and resets variables if the error is low
            print("[!]Stopped")
            self.goalcounter = self.goalcounter + 1
            self.total_distance = 0
            self.previous_distance = 0
           
        diff_distance = distance - self.previous_distance #the derivative error
        #the control signal for distance
        control_signal_distance = kp_distance*distance + ki_distance*self.total_distance + kd_distance*diff_distance
        #applies the control signal from the PID adjustment
        move.linear.x = min(control_signal_distance, 0.2)
        move.angular.z = 0.0
        self.previous_distance = distance #stores the current distance error in to previous distance variable
        self.total_distance = self.total_distance + distance #calculates the integral error
        print("[!]Running %f" % distance)
        self.pub.publish(move)


    def timer_callback(self):
        move = Twist()

        if self.goalcounter == 0: #to move to first goal (0,0) -> (0,1)
            if self.get_flag == 0: #gets the current co-ordinate details
                self.get_flag = 1
                self.currentx = self.my_x
                self.currenty = self.my_y
                print("[!]GOT CURRENT DETAILS: %f %f" %(self.currentx, self.currenty))
            if((self.my_th <=0.7 ) and (self.my_th >= -0.7)): #if the heading is close to 0 degrees
                self.pid_move_controller(self.currentx+1, self.currenty)
            else:
                print("[!]Going to inital heading!")
                self.pid_turn_controller(initial_heading, self.currenty)

        if self.goalcounter == 1: #to move to second goal (0,1) -> (1,1)
            if self.get_flag == 1:
                self.currentx = self.my_x
                self.currenty = self.my_y
                self.get_flag = 2
                print("[!]GOT CURRENT DETAILS: %f %f" %(self.currentx, self.currenty))
            if(self.my_th >=89 and self.my_th <=91): #if heading is close to 90 degrees
                self.pid_move_controller(self.currentx, self.currenty+1)
            else:
                self.pid_turn_controller(final_angle, self.currenty + 1)

        if self.goalcounter == 2:  #to move to third goal (1,1) -> (0,1)
            if self.get_flag == 2:
                self.currentx = self.my_x
                self.currenty = self.my_y
                self.get_flag = 3
                print("[!]GOT CURRENT DETAILS: %f %f" %(self.currentx, self.currenty))
            if((self.my_th >=179 and self.my_th <=180) or (self.my_th >=-180 and self.my_th <= -179)): #if heading is close to 180 degrees
                self.pid_move_controller(self.currentx-1, self.currenty)
            else:
                self.pid_turn_controller(final_angle2, abs(self.currenty+1)) #here, abs() is used to ensure that the rotation is correct

        if self.goalcounter == 3: #to move to final goal (0,1) -> (0,0)
            if self.get_flag == 3:
                self.currentx = self.my_x
                self.currenty = self.my_y
                self.get_flag = 4
                print("[!]GOT CURRENT DETAILS: %f %f" %(self.currentx, self.currenty))
            if (self.my_th <= -89 and self.my_th >= -91): #if angle is close to -90 degrees
                self.pid_move_controller(self.currentx, self.currenty-1)
            else:
                self.pid_turn_controller(final_angle3, abs(self.currenty-1))

        if self.goalcounter == 4: #stop the robot moving, and close the file
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.pub.publish(move)
            print("[!]Path Completed")
            myfile.close
            #rclpy.shutdown()

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

        # else:
        #     if self.goalcounter == 0:
        #         self.pid_controller(3.5, 0.5)
        #     if self.goalcounter == 1:
        #         self.pid_controller(3.5, 3.5)
        #     if self.goalcounter == 2:
        #         self.pid_controller(-2.0, 4.5)
        #     if self.goalcounter == 3:
        #         self.pid_controller(-1.5, -2.0)
        #     if self.goalcounter == 4:
        #         self.pid_controller(1.5, -3.0)
        #     if self.goalcounter == 5:
        #         self.pid_controller(0.0, 0.0)
        #     if self.goalcounter == 6:
        #         move.linear.x = 0.0
        #         move.angular.z = 0.0
        #         self.pub.publish(move)
        #         rclpy.shutdown()