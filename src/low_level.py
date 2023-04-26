#!/usr/bin/env python
import rospy
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

def motor_control(low_cmd_publisher):
    cmd = LowCmd()
    cmd.mode = 0  # Position mode

    # Set desired motor positions, velocities, and torques
    for i in range(12):
        cmd.motor_cmd[i].q = 0  # Desired motor position
        cmd.motor_cmd[i].dq = 0  # Desired motor velocity
        cmd.motor_cmd[i].tau = 0  # Desired motor torque

    cmd.motor_cmd[0].q = 0.5  # Example: Set desired position for motor 0

    low_cmd_publisher.publish(cmd)
    rospy.sleep(0.5)

def control_sequence():
    rospy.init_node('control_sequence', anonymous=True)

    high_cmd_publisher = rospy.Publisher('/unitree_go1/high_cmd', HighCmd, queue_size=1)
    low_cmd_publisher = rospy.Publisher('/unitree_go1/low_cmd', LowCmd, queue_size=1)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        stand_up(high_cmd_publisher)
        rospy.sleep(30)
        
        sit_down(high_cmd_publisher)
        rospy.sleep(30)
        
        motor_control(low_cmd_publisher)
        rospy.sleep(30)

if __name__ == '__main__':
    try:
        control_sequence()
    except rospy.ROSInterruptException:
        pass