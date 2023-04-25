#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist

def calculate_shortest_angle(current_angle, target_angle):
    diff = target_angle - current_angle
    return ((diff + 180) % 360) - 180

def yaw_callback(data):
    global yaw
    yaw = data.data

def position_callback(data):
    global position
    position = data

def check_position():
    global position, last_position , robot_start, position_error_count

    while True:
        current_position = position
        rospy.sleep(0.1)
        if current_position == last_position:
            rospy.logwarn("Position data incorrect, waiting for an update.")
            position_error_count += 1
            if position_error_count >= 100:
                return_function()
            continue
        
        else:
            robot_start = True
            last_position = position
            position_error_count = 0
            break

    return position

def return_function():
    print("React limit. Returning the robot to its original position.")
    global yaw, position, original_position, pub_cmd_vel, running

    running = False

    # Rotate to 0 degrees
    target_yaw = 0
    print("Rotating to 0 degrees")
    while calculate_shortest_angle(yaw, target_yaw) > 5:
        angular_speed = 0.2 if yaw < target_yaw else -0.2
        cmd = Twist()
        cmd.angular.z = angular_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(1)
        

    # Move near original Z position
    target_z = original_position.z
    print("Moving to original Z position")
    while abs(position.z - target_z) > 0.2:
        check_position()
        linear_speed = 0.1 if position.z < target_z else -0.1
        cmd = Twist()
        cmd.linear.y = linear_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(1)
        

    # Move near original X position
    target_x = original_position.x
    print("Moving to original X position")
    while abs(position.x - target_x) > 0.05:
        check_position()
        linear_speed = 0.1 if position.x < target_x else -0.1
        cmd = Twist()
        cmd.linear.x = linear_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(1)
        

    # Stop and wait for shutdown
    print("Back to original position successful")
    cmd = Twist()
    pub_cmd_vel.publish(cmd)
    running = True

def stop_function(event):
    print("Stop function called. Returning the robot to its original position.")
    global yaw, position, original_position, pub_cmd_vel, running

    running = False

    # Rotate to 0 degrees
    target_yaw = 0
    print("Rotating to 0 degrees")
    while calculate_shortest_angle(yaw, target_yaw) > 5:
        angular_speed = 0.2 if yaw < target_yaw else -0.2
        cmd = Twist()
        cmd.angular.z = angular_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(0.1)

    # Move near original Z position
    target_z = original_position.z
    print("Moving to original Z position")
    while abs(position.z - target_z) > 0.2:
        check_position()
        linear_speed = 0.1 if position.z < target_z else -0.1
        cmd = Twist()
        cmd.linear.y = linear_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(0.1)

    # Move near original X position
    target_x = original_position.x
    print("Moving to original X position")
    while abs(position.x - target_x) > 0.05:
        check_position()
        linear_speed = 0.1 if position.x < target_x else -0.1
        cmd = Twist()
        cmd.linear.x = linear_speed
        pub_cmd_vel.publish(cmd)
        rospy.sleep(0.1)

    # Stop and wait for shutdown
    print("Stop and wait for shutdown")
    cmd = Twist()
    pub_cmd_vel.publish(cmd)

def performance_function():
    global pub_cmd_vel, position, running, yaw, limit_x, limit_z
    rate = rospy.Rate(10)  # 10 Hz

    while running:
        # Randomly select a movement
        movement = random.choice(["rotate_left", "rotate_right", "move_left", "move_right", "move_forward", "move_backward"])
        rate.sleep()
        check_position()

        cmd = Twist()
        
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
        
        if position.x >= limit_x[1] or position.x <= limit_x[0] or position.z >= limit_z[1] or position.z <= limit_z[0]:
            print(position)
            return_function()
            continue
        else:
            pub_cmd_vel.publish(cmd)
            rospy.sleep(1)

            # Stop the movement
            cmd = Twist()
            pub_cmd_vel.publish(cmd)

            # Print the movement and position after movement
            
            check_position()

            print("Movement: {}, Position: x={}, z={}".format(movement, position.x, position.z))

            # Rest for a random time between 10 and 60 seconds
            rest_time = random.uniform(10, 60)
            print("Rest for {} sec".format(rest_time))
            rospy.sleep(rest_time)

def main():
    global yaw, position, original_position, pub_cmd_vel, running, robot_start, limit_x, limit_z, position_error_count, last_position

    rospy.init_node("robot_move", anonymous=True)
    rospy.Subscriber("yaw_data", Float64, yaw_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/dog/position", Point, position_callback, queue_size=1, buff_size=2**24)
    pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    print('start subscriber')
    yaw = 0
    position = Point()
    last_position = Point()
    running = True
    robot_start = False
    limit_x = []
    limit_z = []
    position_error_count = 0
    original_position = check_position()

    # Schedule the stop_function to run after 30 minutes
    rospy.Timer(rospy.Duration(1800), stop_function, oneshot=True)

    print('start performance')
    performance_function()
    

if __name__ == "__main__":
    main()