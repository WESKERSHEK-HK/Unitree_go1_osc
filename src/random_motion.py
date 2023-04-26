#!/usr/bin/env python

import rospy
from unitree_legged_msgs.msg import HighCmd
from std_msgs.msg import Header
import time
import random

def main():
    rospy.init_node('random_motion', anonymous=True)

    pub = rospy.Publisher('high_cmd', HighCmd, queue_size=1000)

    loop_rate = rospy.Rate(500)

    motiontime = 0
    high_cmd_ros = HighCmd()

    motions = [
        {
            'description': "Tilting robot body forward",
            'motion': lambda: setattr(high_cmd_ros, 'euler', (lambda x: x, lambda x: x, -0.3))
        },
        {
            'description': "Tilting robot body backward",
            'motion': lambda: setattr(high_cmd_ros, 'euler', (lambda x: x, lambda x: x, 0.3))
        },
        {
            'description': "Tilting robot body left",
            'motion': lambda: setattr(high_cmd_ros, 'euler', (lambda x: x, -0.2, lambda x: x))
        },
        {
            'description': "Tilting robot body right",
            'motion': lambda: setattr(high_cmd_ros, 'euler', (lambda x: x, 0.2, lambda x: x))
        },
        {
            'description': "Lowering robot body",
            'motion': lambda: setattr(high_cmd_ros, 'bodyHeight', -0.2)
        },
        {
            'description': "Raising robot body",
            'motion': lambda: setattr(high_cmd_ros, 'bodyHeight', 0.1)
        }
    ]

    def reset_motion():
        setattr(high_cmd_ros, 'euler', (0.0, 0.0, 0.0))
        setattr(high_cmd_ros, 'bodyHeight', 0.0)

    while not rospy.is_shutdown():

        # Select a random motion and execute it
        selected_motion = random.choice(motions)
        print(selected_motion['description'])
        selected_motion['motion']()

        high_cmd_ros.header.stamp = rospy.Time.now()
        high_cmd_ros.levelFlag = 1
        high_cmd_ros.mode = 1

        # Reset other motion parameters
        high_cmd_ros.gaitType = 0
        high_cmd_ros.speedLevel = 0
        high_cmd_ros.footRaiseHeight = 0
        high_cmd_ros.velocity.x = 0.0
        high_cmd_ros.velocity.y = 0.0
        high_cmd_ros.yawSpeed = 0.0
        high_cmd_ros.reserve = 0

        pub.publish(high_cmd_ros)
        loop_rate.sleep()

        # Reset motion to neutral position
        print("Resetting motion to neutral position")
        reset_motion()

        high_cmd_ros.header.stamp = rospy.Time.now()
        pub.publish(high_cmd_ros)
        loop_rate.sleep()

        # Delay between 10 to 30 seconds
        delay = random.uniform(10, 30)
        print("Delaying for {} seconds".format(delay))
        time.sleep(delay)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass