#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios
from pythonosc import dispatcher, osc_server
import threading

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def process_keyboard_input(pub):
    twist = Twist()

    while not rospy.is_shutdown():
        ch = getch()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        if ch == 'q':
            break
        elif ch == 'w':
            twist.linear.x = 0.5
        elif ch == 's':
            twist.linear.x = -0.5
        elif ch == 'a':
            twist.linear.y = 0.5
        elif ch == 'd':
            twist.linear.y = -0.5
        elif ch == 'j':
            twist.angular.z = 1.0
        elif ch == 'l':
            twist.angular.z = -1.0

        pub.publish(twist)

def handle_osc_message(unused_addr, args, value):
    pub, command = args
    twist = Twist()

    if command == 'forward':
        twist.linear.x = 0.5
    elif command == 'backward':
        twist.linear.x = -0.5
    elif command == 'left':
        twist.linear.y = 0.5
    elif command == 'right':
        twist.linear.y = -0.5
    elif command == 'turn_left':
        twist.angular.z = 1.0
    elif command == 'turn_right':
        twist.angular.z = -1.0

    pub.publish(twist)

def main():
    rospy.init_node('keyboard_input_node', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    disp = dispatcher.Dispatcher()
    disp.map("/forward", handle_osc_message, (pub, 'forward'))
    disp.map("/backward", handle_osc_message, (pub, 'backward'))
    disp.map("/left", handle_osc_message, (pub, 'left'))
    disp.map("/right", handle_osc_message, (pub, 'right'))
    disp.map("/turn_left", handle_osc_message, (pub, 'turn_left'))
    disp.map("/turn_right", handle_osc_message, (pub, 'turn_right'))

    server = osc_server.ThreadingOSCUDPServer(('192.168.3.133', 10000), disp)

    keyboard_thread = threading.Thread(target=process_keyboard_input, args=(pub,))
    keyboard_thread.start()

    rospy.loginfo("Serving on %s:%s", server.server_address[0], server.server_address[1])
    server.serve_forever()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass