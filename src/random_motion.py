#!/usr/bin/env python

import rospy
from unitree_legged_msgs.msg import HighCmd
from std_msgs.msg import Header
import time
import random

def random_motion(high_cmd_ros):
    motions = ['tilt_forward', 'tilt_backward', 'tilt_left', 'tilt_right', 'sit', 'raise']
    target_motion = random.choice(motions)
    if target_motion == 'tilt_forward':
        high_cmd_ros.euler[0] = -0.3
        high_cmd_ros.bodyHeight = 0.0
    elif target_motion == 'tilt_backward':
        high_cmd_ros.euler[0] = 0.3
        high_cmd_ros.bodyHeight = 0.0
    elif target_motion == 'tilt_left':
        high_cmd_ros.euler[1] = -0.2
        high_cmd_ros.bodyHeight = 0.0
    elif target_motion == 'tilt_right':
        high_cmd_ros.euler[1] = 0.2
        high_cmd_ros.bodyHeight = 0.0
    elif target_motion == 'sit':
        high_cmd_ros.bodyHeight = -0.2
    elif target_motion == 'raise':
        high_cmd_ros.bodyHeight = 0.1

    print("Random Motion: {}".format(target_motion))
    return high_cmd_ros


def reset_motion(selected_motion):
    high_cmd_ros = selected_motion
    high_cmd_ros.euler[0] = 0.0
    high_cmd_ros.euler[1] = 0.0
    high_cmd_ros.bodyHeight = 0.0

    return high_cmd_ros

def main():
    rospy.init_node('random_motion', anonymous=True)

    pub = rospy.Publisher('high_cmd', HighCmd, queue_size=1000)

    loop_rate = rospy.Rate(1000)

    motiontime = 0
    high_cmd_ros = HighCmd()

    while not rospy.is_shutdown():

        #high_cmd_ros.header.stamp = rospy.Time.now()
        #high_cmd_ros.head[0] = '0xFE'
        #high_cmd_ros.head[1] = '0xEF'
        high_cmd_ros.levelFlag = 1
        high_cmd_ros.mode = 1

        # Reset other motion parameters
        high_cmd_ros.gaitType = 0
        high_cmd_ros.speedLevel = 0
        high_cmd_ros.footRaiseHeight = 0
        high_cmd_ros.velocity[0]= 0.0
        high_cmd_ros.velocity[1]= 0.0
        high_cmd_ros.yawSpeed = 0.0
        high_cmd_ros.reserve = 0

        # Select a random motion and execute it
        selected_motion = random_motion(high_cmd_ros)
        print("Motion: {}".format(selected_motion))
        count = 0
        while count < 1000:
            count += 2
            pub.publish(selected_motion)
        
        # Reset motion to neutral position
        print("Resetting motion to neutral position")
        ori_motion = reset_motion(selected_motion)

        print("Motion: {}".format(selected_motion))
        pub.publish(ori_motion)
        
        count = 0
        while count < 1000:
            count += 2
            pub.publish(selected_motion)

        # Delay between 10 to 30 seconds
        delay = random.uniform(10, 30)
        print("Delaying for {} seconds".format(delay))
        time.sleep(delay)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass