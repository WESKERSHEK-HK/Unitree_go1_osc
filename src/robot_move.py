#!/usr/bin/env python

import rospy
import serial
import math
import random
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Point, Twist


current_yaw = 0
robot_home = False
robot_turn = 0

def yaw_callback(yaw_data):
    global current_yaw
    current_yaw = yaw_data.data


def home_callback(home_data):
    global robot_home
    robot_home = True

def performance():
    global current_yaw, robot_home, pub_cmd_vel, robot_turn

    angles = [270,60]
    #rospy.loginfo("current_yaw: %f ", current_yaw)
    if robot_home == False:
        twist = Twist()
        if robot_turn == 0:
            if current_yaw >= 265 and current_yaw <= 275:
                twist.angular.z = 0.0
                pub_cmd_vel.publish(twist)
                robot_turn = 1
                rest_sec = random.randint(20,60)
                rospy.loginfo("Rest for: %f sec", rest_sec)
                rospy.sleep(rest_sec)
                print("react 270 degree, start turning back")
            else:
                twist.angular.z = 0.2
                pub_cmd_vel.publish(twist)
        else:
            if current_yaw >= 55 and current_yaw <= 65:
                twist.angular.z = 0.0
                pub_cmd_vel.publish(twist)
                robot_turn = 0
                rest_sec = random.randint(20,60)
                rospy.loginfo("Rest for: %f sec", rest_sec)
                rospy.sleep(rest_sec)
                print("react 60 degree, start turning back")
            else:
                twist.angular.z = -0.2
                pub_cmd_vel.publish(twist)
        rospy.sleep(2)
        twist.angular.z = 0.0
        pub_cmd_vel.publish(twist)
        print("stop Turning")

def main():
    global pub_cmd_vel, pub_home_done

    rospy.init_node("robot_move", anonymous=True)
    rospy.Subscriber("yaw_data", Float64, yaw_callback)
    rospy.Subscriber("/dog/home", Empty, home_callback)

    pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    pub_home_done = rospy.Publisher("/dog/homedone", Empty, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz

    print('start performance mode')
    performance()


if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()
    else:
        pass