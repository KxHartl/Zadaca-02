import math
import numpy as np 
from time import sleep 
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class Bug2(Node):
    #Initialization
    def __init__(self):
        super().__init__('bug2')

        # subsriberi
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)

        # brzine
        self.forward_speed = 0.1
        self.turning_speed = 0.6

        self.turning_speed_yaw_adjustment = 0.08

        self.turning_speed_wf_fast = 0.8
        self.turning_speed_wf_slow = 0.35

        # tolerancije
        self.yaw_precision = 2 *(math.pi / 180)
        self.dist_precision = 0.1 # tolerancija za cilj
        self.distance_to_start_goal_line_precision = 0.1

        self.leave_point_to_hit_point_diff = 0.2

        # limiti
        self.dist_thresh_obs = 0.5 #udaljenost za trazenje prepreka
        self.dist_thresh_wf = 0.45 #udaljenost na kojoj se prati zid
        self.dist_too_close_to_wall = 0.4 #je li je robot pre blizu yidu
        self.dist_thresh_bug2 = 0.5 #udaljenost kad je robot dosao do zida

        # varijable senzora
        self.leftfront_dist = 0.0
        self.rightfront_dist = 0.0

        # trenutna pozicja
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # kordinate cilja
        self.goal_x_coordinates = False
        self.goal_y_coordinates = False

        self.goal_idx = 0
        self.goal_max_idx =  None

        # mod rada
        self.robot_mode = "go to goal mode"

        self.go_to_goal_state = "adjust heading"
        self.wall_following_state = "turn left"
        
        # stanje
        self.bug2_switch = "ON"
        self.start_goal_line_calculated = False

        # Start-Goal Line Parameters
        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0

        # varijable za zid
        self.hit_point_x = 0
        self.hit_point_y = 0

        self.leave_point_x = 0
        self.leave_point_y = 0

        self.distance_to_goal_from_hit_point = 0.0
        self.distance_to_goal_from_leave_point = 0.0
        
        # debug varijable
        self.brojac = 0
        self.dodris_linija = 0

        # poruke
        self.poruka_1 = ""
        self.poruka_2 = ""
        self.poruka_3 = ""



    #Method for goal update
    def goal_callback(self, msg):
        self.goal_x_coordinates = msg.pose.position.x
        self.goal_y_coordinates = msg.pose.position.y

    #Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_yaw = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    #Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.leftfront_dist = msg.range

    def fr_sensor_callback(self, msg):
        self.rightfront_dist = msg.range


    def bug_algorithm_callback(self):
        # print("Current X: " , self.current_x)
        # print("Current Y: " , self.current_y)
        # print("Current theta: " , self.current_yaw)
        # print("FL Sensor: " , self.leftfront_dist)
        # print("FR Sensor: " , self.rightfront_dist)
        # print("Goal X: " , self.goal_x)
        # print("Goal Y: " , self.goal_y)
        print("NOVA PORUKA")
        print(self.poruka_1)
        print(self.poruka_2)
        print(self.poruka_3)
        print("")

        if self.robot_mode == "obstacle avoidance mode":
            self.avoid_obstacles()
        
        if self.goal_x_coordinates == False and self.goal_y_coordinates == False:
            return
        
        self.bug2()

    def avoid_obstacles(self):

        self.poruka_1 = "METODA: avoid_obstacles"

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
         

        d = self.dist_thresh_obs

        if   self.leftfront_dist > d and self.rightfront_dist > d:
            msg.linear.x = self.forward_speed   #naprijed

        elif self.leftfront_dist > d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed  #rotacija lijevo

        elif self.leftfront_dist < d and self.rightfront_dist > d:
            msg.angular.z = -self.turning_speed # rotacija desno

        elif self.leftfront_dist < d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed  ##rotacija lijevo

        else:
            pass
             
        
        self.cmd_pub.publish(msg)

    def go_to_goal(self):

        self.poruka_1 = "METODA: go to goal 0000000000000000000000000000000000000000000"

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
    
        d = self.dist_thresh_bug2
        d_2 = self.dist_too_close_to_wall

        if (self.leftfront_dist < d or self.rightfront_dist < d_2): #robot je naisao na zid

            self.poruka_3 = "PORUKA: ROBOT JE NAISAO NA ZID"
        
            self.robot_mode = "wall following mode"
            self.poruka_2 = "ROBOT_MODE = wall following mode"
            
            # Record the hit point  
            self.hit_point_x = self.current_x
            self.hit_point_y = self.current_y
            
            # Record the distance to the goal from the 
            # hit point
            self.distance_to_goal_from_hit_point = math.sqrt(
                (pow(self.goal_x_coordinates - self.hit_point_x, 2)) +
                (pow(self.goal_y_coordinates - self.hit_point_y, 2)))  

            msg.linear.x = 0.0   
            msg.angular.z = self.turning_speed_wf_fast +1 #rotacija lijevo
                    
            self.cmd_pub.publish(msg)
                   
            return
             
        # Fix the heading       
        if (self.go_to_goal_state == "adjust heading"):

            self.poruka_3 = "PORUKA: ROBOT SE USMJERAVA"
             
            # Calculate the desired heading based on the current position 
            # and the desired position
            desired_yaw = math.atan2(
                    self.goal_y_coordinates - self.current_y,
                    self.goal_x_coordinates - self.current_x)
             
            # How far off is the current heading in radians?        
            yaw_error = desired_yaw - self.current_yaw
             
            # Adjust heading if heading is not good enough
            if math.fabs(yaw_error) > self.yaw_precision:
             
                if yaw_error > 0:          
                    msg.angular.z = self.turning_speed_yaw_adjustment   #rotiranje lijevo
                else:
                    msg.angular.z = -self.turning_speed_yaw_adjustment  #rotiranje desno
                 
                self.cmd_pub.publish(msg)
                 
            # Change the state if the heading is good enough
            else:               
                self.go_to_goal_state = "go straight"
                self.poruka_2 = "ROBOT_MODE = go to goal // go straight"
                 
                # Command the robot to stop turning
                self.cmd_pub.publish(msg)        
 
        # Go straight                                       
        elif (self.go_to_goal_state == "go straight"):
             
            position_error = math.sqrt(
                pow(self.goal_x_coordinates - self.current_x, 2) + 
                pow(self.goal_y_coordinates - self.current_y, 2))
                         
             
            # If we are still too far away from the goal                        
            if position_error > self.dist_precision:
 
                msg.linear.x = self.forward_speed
                     
                self.cmd_pub.publish(msg)
             
                # Check our heading         
                desired_yaw = math.atan2(
                    self.goal_y_coordinates - self.current_y,
                    self.goal_x_coordinates - self.current_x)
                 
                # How far off is the heading?   
                yaw_error = desired_yaw - self.current_yaw      
         
                # Check the heading and change the state if there is too much heading error
                if math.fabs(yaw_error) > self.yaw_precision:
                     
                    # Change the state
                    self.go_to_goal_state = "adjust heading"
                    self.poruka_2 = "ROBOT_MODE = go to goal // adjust heading"

                 
            # We reached our goal. Change the state.
            else:           
                # Change the state
                self.go_to_goal_state = "goal achieved"
                self.poruka_2 = "ROBOT_MODE = go to goal // goal achieved"

                self.robot_mode = "done"
                
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)


         
        # Goal achieved         
        elif (self.go_to_goal_state == "goal achieved"):
         
            self.start_goal_line_calculated = False            
         
        else:

            pass

    
    def follow_wall(self):

        self.poruka_1 = "METODA: follow wall"
        self.poruka_3 = "PORUKA: "

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0   
        
        # Calculate the point on the start-goal 
        # line that is closest to the current position
        x_start_goal_line = self.current_x
        y_start_goal_line =(self.start_goal_line_slope_m *(x_start_goal_line)) +(self.start_goal_line_y_intercept)
                        
        # Calculate the distance between current position 
        # and the start-goal line
        distance_to_start_goal_line = math.sqrt(
            pow(x_start_goal_line - self.current_x, 2) + 
            pow(y_start_goal_line - self.current_y, 2)) 
                            
        # If we hit the start-goal line again               
        if distance_to_start_goal_line < self.distance_to_start_goal_line_precision:

            self.poruka_3 = "PORUKA: DOSLI SMO DO LINIJE"

            # Determine if we need to leave the wall and change the mode
            # to 'go to goal'
            # Let this point be the leave point
            self.leave_point_x = self.current_x
            self.leave_point_y = self.current_y

            # Record the distance to the goal from the leave point
            self.distance_to_goal_from_leave_point = math.sqrt(
                pow(self.goal_x_coordinates - self.leave_point_x, 2) + 
                pow(self.goal_y_coordinates - self.leave_point_y, 2)) 
            
            # Is the leave point closer to the goal than the hit point?
            # If yes, go to goal. 
            diff = self.distance_to_goal_from_hit_point - self.distance_to_goal_from_leave_point
            self.poruka_3 = "PORUKA: DOSLI SMO DO LINIJE    diff = " +str(diff)

            if diff > self.leave_point_to_hit_point_diff:

                self.poruka_3 = "PORUKA: ZELIMO ICI DO CILJA diff = " +str(diff)

                # Change the mode. Go to goal.
                self.robot_mode = "go to goal mode"
                self.poruka_2 = "ROBOT_MODE = go to goal"

                self.go_to_goal_state = "adjust heading"
                self.poruka_2 = "ROBOT_MODE = go to goal // adjust heading"

                msg.linear.x = 0.0
                msg.angular.z = self.turning_speed_wf_fast +2 #rotacija lijevo
                    
                self.cmd_pub.publish(msg)

                return             
         
        d = self.dist_thresh_wf
        d_2 = self.dist_too_close_to_wall
         
        if self.leftfront_dist > d and self.rightfront_dist > d:
            #skretanje desno
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow +0.05
             
        elif (self.leftfront_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < self.dist_too_close_to_wall):

                #skretanje lijevo
                msg.linear.x = self.forward_speed
                msg.angular.z = self.turning_speed_wf_fast
            else:           

                #nastavi ravno
                msg.linear.x = self.forward_speed
                                     
        elif self.leftfront_dist < d_2 and self.rightfront_dist > d:

            # msg.linear.x = 0
            msg.angular.z = self.turning_speed_wf_slow
        
        elif self.leftfront_dist < d and self.rightfront_dist > d:

            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow +0.15
              
        elif self.leftfront_dist < d and self.rightfront_dist < d:
            # msg.linear.x = 0
            msg.angular.z = self.turning_speed_wf_slow

        else:
            pass

        self.cmd_pub.publish(msg)

    def bug2(self):
     
        # Each time we start towards a new goal, we need to calculate the start-goal line
        if self.start_goal_line_calculated == False:
         
            # Make sure go to goal mode is set.
            self.robot_mode = "go to goal mode"            
 
            self.start_goal_line_xstart = self.current_x
            self.start_goal_line_xgoal = self.goal_x_coordinates
            self.start_goal_line_ystart = self.current_y
            self.start_goal_line_ygoal = self.goal_y_coordinates
             
            # Calculate the slope of the start-goal line m
            self.start_goal_line_slope_m = (
                (self.start_goal_line_ygoal - self.start_goal_line_ystart) / 
                (self.start_goal_line_xgoal - self.start_goal_line_xstart))
             
            # Solve for the intercept b
            self.start_goal_line_y_intercept = (
                self.start_goal_line_ygoal - (self.start_goal_line_slope_m * self.start_goal_line_xgoal))
                 
            # We have successfully calculated the start-goal line
            self.start_goal_line_calculated = True
             
        if self.robot_mode == "go to goal mode":
            self.go_to_goal()     

        elif self.robot_mode == "wall following mode":
            self.follow_wall()

        elif self.robot_mode == "done":
            pass


def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug2()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
