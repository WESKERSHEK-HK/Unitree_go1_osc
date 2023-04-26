#!/usr/bin/env python
import rospy
import random
from unitree_legged_msgs.msg import HighCmd, LowCmd

def sit_down(cmd_publisher):
    cmd = HighCmd()
    cmd.mode = 1  # Position mode

    # Set desired joint angles for sitting down
    cmd.q1 = [0, -0.8, 1.6, 0, -0.8, 1.6, 0, -0.8, 1.6, 0, -0.8, 1.6]
    cmd.duration = 2000  # Time duration in milliseconds

    cmd_publisher.publish(cmd)
    rospy.sleep(2.5)

def stand_up(cmd_publisher):
    cmd = HighCmd()
    cmd.mode = 1  # Position mode

    # Set desired joint angles for standing up
    cmd.q1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    cmd.duration = 2000  # Time duration in milliseconds

    cmd_publisher.publish(cmd)
    rospy.sleep(2.5)

def motor_control(cmd_publisher, leg_index, duration):
    cmd = HighCmd()
    cmd.mode = 1  # Position mode

    # Set desired joint angles for the specified leg
    target_position = [0, -0.8, 1.6] if leg_index % 2 == 0 else [0, 0.8, -1.6]
    for i in range(3):
        cmd.q1[leg_index * 3 + i] = target_position[i]
    
    cmd.duration = duration  # Time duration in milliseconds

    cmd_publisher.publish(cmd)
    rospy.sleep(duration / 1000.0 + 0.5)

def control_sequence():
    rospy.init_node('control_sequence', anonymous=True)

    cmd_publisher = rospy.Publisher('/unitree_go1/high_cmd', HighCmd, queue_size=1)
    rospy.sleep(0.5)

    actions = [stand_up, sit_down, motor_control]
    front_legs = [0, 2]  # Indices of the front legs

    while not rospy.is_shutdown():
        random.shuffle(actions)

        for action in actions:
            if action == motor_control:
                leg_index = random.choice(front_legs)
                duration = random.randint(2000, 4000)  # Random duration between 2 and 4 seconds
                action(cmd_publisher, leg_index, duration)
            else:
                action(cmd_publisher)

            rospy.sleep(30)

if __name__ == '__main__':
    try:
        control_sequence()
    except rospy.ROSInterruptException:
        pass