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
    global position

    while not rospy.is_shutdown():
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
    global yaw, position, original_position, pub_cmd_vel

    # Rotate to 0 degrees
    target_yaw = 0
    while abs(yaw - target_yaw) > 0.01:
        print("Rotating to 0 degrees")
        angular_speed = 0.2 if yaw < target_yaw else -0.2
        cmd = Twist()
        cmd.angular.z = angular_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(0.1)

    # Move near original Z position
    target_z = original_position.z
    while abs(position.z - target_z) > 50:
        print("Moving to original Z position")
        check_position()
        linear_speed = 0.1 if position.z < target_z else -0.1
        cmd = Twist()
        cmd.linear.y = linear_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(0.1)

    # Move near original X position
    target_x = original_position.x
    while abs(position.x - target_x) > 50:
        print("Moving to original X position")
        check_position()
        linear_speed = 0.1 if position.x < target_x else -0.1
        cmd = Twist()
        cmd.linear.x = linear_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(0.1)

    # Stop and wait for shutdown
    print("Stop Function Done")
    cmd = Twist()
    pub_cmd_vel.publish(cmd)

def performance_function():
    global pub_cmd_vel, position

    while not rospy.is_shutdown():
        # Randomly select a movement
        movement = random.choice(["rotate_left", "rotate_right", "move_left", "move_right", "move_forward", "move_backward"])

        cmd = Twist()

        # Check if the movement would exceed limits
        if movement == "move_left" and position.z >= 1300:
            continue
        elif movement == "move_right" and position.z <= 1000:
            continue
        elif movement == "move_forward" and position.x >= 750:
            continue
        elif movement == "move_backward" and position.x <= 550:
            continue

        # Perform the movement for 1 second
        if movement == "rotate_left":
            cmd.angular.z = 0.2
        elif movement == "rotate_right":
            cmd.angular.z = -0.2
        elif movement == "move_left":
            cmd.linear.y = 0.1
        elif movement == "move_right":
            cmd.linear.y = -0.1
        elif movement == "move_forward":
            cmd.linear.x = 0.1
        elif movement == "move_backward":
            cmd.linear.x = -0.1

        pub_cmd_vel.publish(cmd)
        rospy.sleep(1)

        # Stop the movement
        cmd = Twist()
        pub_cmd_vel.publish(cmd)

        # Print the movement and position after movement
        print(f"Movement: {movement}, Position: x={position.x}, z={position.z}")

        # Rest for a random time between 10 and 60 seconds
        rest_time = random.uniform(10, 60)
        rospy.sleep(rest_time)

rospy.init_node("robot_move", anonymous=True)
rospy.Subscriber("yaw_data", Float64, yaw_callback)
rospy.Subscriber("/dog/position", Point, position_callback)
pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

yaw = 0
position = Point()
original_position = check_position()

# Schedule the stop_function to run after 30 minutes
rospy.Timer(rospy.Duration(1800), stop_function, oneshot=True)

performance_function()