#!/usr/bin/env python

import rospy
import serial
import math
import random
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Point, Twist

global current_yaw,robot_home,robot_turn,robot_rotate_to_zero,current_position
current_yaw = 0
robot_home = False
robot_turn = 0
robot_rotate_to_zero = False
current_position = Point()

def position_callback(data):
    global current_position, robot_start, robot_home
    current_position.x = data.x
    current_position.y = data.z
    if not robot_home:
        if current_position.x > 700 or current_position.x < 600:
            robot_home = True
   


def yaw_callback(yaw_data):
    global current_yaw
    current_yaw = yaw_data.data


def home_callback(home_data):
    global robot_home
    robot_home = True

def stop_timer_callback():
    global robot_home
    robot_home = True

def calculate_shortest_angle(current_angle, target_angle):
    diff = target_angle - current_angle
    return ((diff + 180) % 360) - 180

def back_to_origin():
    global current_yaw, robot_home, pub_cmd_vel, robot_turn, robot_rotate_to_zero
    diff_angle = calculate_shortest_angle(current_yaw, 0)
    direction = 0
    current_yaw = int(current_yaw)
    while not robot_rotate_to_zero:
        twist = Twist()
        if diff_angle > 0:
            direction = 1
        else:
            direction = -1

        if current_yaw > 355.0 or current_yaw < 5.0:
            twist.angular.z = 0.0
            pub_cmd_vel.publish(twist)
            robot_rotate_to_zero = True
            #print('Done rotate back to origin')
            break
        else:
            twist.angular.z = 0.2 * direction
            pub_cmd_vel.publish(twist)
            rospy.loginfo("Going back to zero, Current angle: %f degree", current_yaw)
            
        #rospy.sleep(0.1)
    

def move_to_origin():
    global current_position,robot_home,pub_cmd_vel
    twist = Twist()
    while robot_home:
        rospy.loginfo("move_to_origin")
        if current_position.x >= 700:
            twist.linear.x = 0.1
        elif current_position.x <= 	600:
            twist.linear.x = -0.1
        else:
            twist.linear.x = 0.0
            robot_home = False
        pub_cmd_vel.publish(twist)

    

def performance():
    global current_yaw, robot_home, pub_cmd_vel, robot_turn

    angles = [300,30]
    #rospy.loginfo("current_yaw: %f ", current_yaw)
    if not robot_home:
        twist = Twist()
        if robot_turn == 0:
            if current_yaw < 300.0 and current_yaw > 180.0:
                twist.angular.z = 0.0
                pub_cmd_vel.publish(twist)
                robot_turn = 1
                
                print("react 300 degree, start turning back")
            else:
                twist.angular.z = 0.2
                pub_cmd_vel.publish(twist)
        else:
            if current_yaw > 30.0 and current_yaw < 180.0:
                twist.angular.z = 0.0
                pub_cmd_vel.publish(twist)
                robot_turn = 0
                
                print("react 30 degree, start turning back")
            else:
                twist.angular.z = -0.2
                pub_cmd_vel.publish(twist)

        rospy.sleep(2)
        twist.angular.z = 0.0
        twist.linear.x = -0.1
        pub_cmd_vel.publish(twist)
        rospy.sleep(0.5)
        twist.linear.x = 0.0
        pub_cmd_vel.publish(twist)

        if current_yaw < 300.0 and current_yaw > 180.0:
                twist.angular.z = 0.0
                pub_cmd_vel.publish(twist)
                robot_turn = 1
                
                print("react 300 degree, start turning back")

        if current_yaw > 30.0 and current_yaw < 180.0:
                twist.angular.z = 0.0
                pub_cmd_vel.publish(twist)
                robot_turn = 0
                
                print("react 30 degree, start turning back")

        rest_sec = random.randint(5,10)
        rospy.loginfo("Current angle: %f degree", current_yaw)
        rospy.loginfo("Rest for: %f sec", rest_sec)
        rospy.sleep(rest_sec)
    else:
        back_to_origin()

def main():
    global pub_cmd_vel, pub_home_done, current_position, robot_rotate_to_zero

    rospy.init_node("robot_move", anonymous=True)
    rospy.Subscriber("yaw_data", Float64, yaw_callback)
    rospy.Subscriber("/dog/home", Empty, home_callback)
    rospy.Subscriber("/dog/position", Point, position_callback)

    pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    pub_home_done = rospy.Publisher("/dog/homedone", Empty, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz
    current_position = Point()
    robot_rotate_to_zero = False
    timer = rospy.Timer(rospy.Duration(1800), stop_timer_callback, oneshot=True)

    #print('start performance mode')
    performance()


if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()
    else:
        pass
