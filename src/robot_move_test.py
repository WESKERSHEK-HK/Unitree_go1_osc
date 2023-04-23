#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist

def yaw_callback(data):
    global yaw
    yaw = data.data

def position_callback(data):
    global position
    position = data

def check_position():
    global position, running

    while running:
        current_position = position
        rospy.sleep(0.1)
        if current_position == position:
            continue
        elif position.z == 0.0 or abs(position.z - current_position.z) > 300:
            rospy.logwarn("Position data incorrect, waiting for an update.")
        else:
            break

    return position

def stop_function(event):
    print("Stop function called. Returning the robot to its original position.")
    global yaw, position, original_position, pub_cmd_vel, running

    running = False

    # Rest of the stop_function code remains unchanged

def performance_function():
    global pub_cmd_vel, position, running

    while running:
        # Rest of the performance_function code remains unchanged

def main():
    global yaw, position, original_position, pub_cmd_vel, running

    rospy.init_node("robot_move", anonymous=True)
    rospy.Subscriber("yaw_data", Float64, yaw_callback)
    rospy.Subscriber("/dog/position", Point, position_callback)
    pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    yaw = 0
    position = Point()
    running = True
    original_position = check_position()

    # Schedule the stop_function to run after 30 minutes
    rospy.Timer(rospy.Duration(1800), stop_function, oneshot=True)
    print('start')
    performance_function()

if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()
    else:
        pass