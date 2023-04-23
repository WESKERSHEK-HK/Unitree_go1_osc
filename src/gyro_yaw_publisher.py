#!/usr/bin/env python

import rospy
import serial
import math
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Point, Twist

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

current_position = Point()
current_yaw = 0
origin = Point()
robot_start = False
robot_home = False
robot_turn = 0

def origin_setup(data):
    global origin, robot_start
    origin.x = data.x
    origin.y = data.y
    rospy.loginfo("Received origin: x = %f, y = %f", data.x, data.y)
    robot_start = True
    
def position_callback(data):
    global current_position, robot_start
    current_position.x = data.x
    current_position.y = data.y
    #rospy.loginfo("Received position: x = %f, y = %f", data.x, data.y)
    if robot_start == False:
        origin_setup(current_position)

def stop_callback(data):
    global current_yaw, robot_home
    if robot_home == False:
        robot_home = True
        angle_to_origin = calculate_angle_to_origin(current_position)
        rospy.loginfo("Angle to origin: %f degrees", angle_to_origin)

        shortest_angle = calculate_shortest_angle(current_yaw, angle_to_origin)
        turn_direction = decide_turn_direction(shortest_angle)
        execute_turn(pub_cmd_vel, shortest_angle, turn_direction)

        move_forward_to_origin(pub_cmd_vel)

        turn_to_zero_degrees(pub_cmd_vel)

        

def home_callback(data):
    global current_yaw, robot_home ,pub_cmd_vel
    if robot_home == False:
        robot_home = True
        angle_to_origin = calculate_angle_to_origin(current_position)
        rospy.loginfo("Angle to origin: %f degrees", angle_to_origin)

        shortest_angle = calculate_shortest_angle(current_yaw, angle_to_origin)
        turn_direction = decide_turn_direction(shortest_angle)
        execute_turn(pub_cmd_vel, shortest_angle, turn_direction)

        move_forward_to_origin(pub_cmd_vel)

        turn_to_zero_degrees(pub_cmd_vel)

        pub_home_done.publish(Empty())

        robot_home = False

def calculate_angle_to_origin(position):
    dx = position.x - origin.x
    dy = position.y - origin.y
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def calculate_shortest_angle(current_angle, target_angle):
    diff = target_angle - current_angle
    return ((diff + 180) % 360) - 180

def decide_turn_direction(shortest_angle):
    if shortest_angle > 0:
        return 1  # Turn left
    else:
        return -1  # Turn right

def execute_turn(pub_cmd_vel, shortest_angle, turn_direction, tolerance=5, angular_speed=0.2):
    global current_yaw

    target_yaw = current_yaw + shortest_angle
    target_yaw = (target_yaw + 360) % 360

    twist = Twist()
    twist.angular.z = turn_direction * angular_speed

    rospy.loginfo("Turning to face target angle...")

    while not rospy.is_shutdown():
        angle_error = calculate_shortest_angle(current_yaw, target_yaw)

        if abs(angle_error) <= tolerance:
            break

        pub_cmd_vel.publish(twist)
        rospy.sleep(0.3)

    twist.angular.z = 0
    pub_cmd_vel.publish(twist)
    rospy.loginfo("Turn completed.")

def move_forward_to_origin(pub_cmd_vel, tolerance=0.1, linear_speed=0.2):
    rospy.loginfo("Moving forward to origin...")

    twist = Twist()
    twist.linear.x = linear_speed

    while not rospy.is_shutdown():
        dx = current_position.x - origin.x
        dy = current_position.y - origin.y
        distance_to_origin = math.sqrt(dx**2 + dy**2)

        if distance_to_origin <= tolerance:
            break

        pub_cmd_vel.publish(twist)
        rospy.sleep(0.1)

    twist.linear.x = 0
    pub_cmd_vel.publish(twist)
    rospy.loginfo("Reached origin.")

def sit_down_to_stop(pub_cmd_vel, tolerance=0.1, linear_speed=0.2):
    rospy.loginfo("Stop the robot...")

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    pub_cmd_vel.publish(twist)
    rospy.sleep(2)
    
    rospy.loginfo("Waiting to shutdown.")

def turn_to_zero_degrees(pub_cmd_vel, tolerance=5, angular_speed=0.1):
    rospy.loginfo("Turning to face 0 degrees...")

    shortest_angle = calculate_shortest_angle(current_yaw, 0)
    turn_direction = decide_turn_direction(shortest_angle)

    execute_turn(pub_cmd_vel, shortest_angle, turn_direction, tolerance, angular_speed)

    rospy.loginfo("Facing 0 degrees.")

def main():
    global pub_cmd_vel, pub_home_done, current_yaw

    rospy.init_node("gyro_yaw_publisher", anonymous=True)
    rospy.Subscriber("/dog/position", Point, position_callback)
    rospy.Subscriber("/dog/home", Empty, home_callback)
    rospy.Subscriber("/dog/stop", Empty, stop_callback)

    pub = rospy.Publisher("yaw_data", Float64, queue_size=1)
    pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    pub_home_done = rospy.Publisher("/dog/homedone", Empty, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        data = str(ser.readline())
        split_data = data.split(", ")
        yaw = None
        #print(data)
        try:
            for item in split_data:
                key, value = item.split(": ")
                #print(value)
                if "yaw" in key:
                    print(item)
                    yaw = float(value[0:2])
                    if yaw < 0:
                        yaw += 360

                    current_yaw = yaw
                    print(yaw)
                    pub.publish(yaw)  # Publish the yaw value

        except Exception as e:
            pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
