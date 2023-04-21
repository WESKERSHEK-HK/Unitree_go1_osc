#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
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
    global paused
    twist = Twist()

    while not rospy.is_shutdown():
        if not paused:
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
            elif ch == 'i':
                twist.angular.y = 0.0
            elif ch == 'm':
                twist.angular.y = -0.2

            pub.publish(twist)

def homedone_callback(msg):
    global paused
    print('Received homedone message')
    paused = False

def handle_home_osc_message(unused_addr, args):
    global home_pub, paused
    paused = True
    empty_msg = Empty()
    print('Received osc command: home')
    home_pub.publish(empty_msg)

def handle_osc_message(unused_addr, args, value):
    global pub, paused
    if not paused:
        command = args
        twist = Twist()

        if command == 'forward':
            twist.linear.x = 0.1
        elif command == 'backward':
            twist.linear.x = -0.1
        elif command == 'left':
            twist.linear.y = 0.1
        elif command == 'right':
            twist.linear.y = -0.1
        elif command == 'turn_left':
            twist.angular.z = 0.1
        elif command == 'turn_right':
            twist.angular.z = -0.1
        elif command == 'stand':
            twist.angular.y = 0.0
        elif command == 'sit':
            twist.angular.y = -0.2
        print('Received osc command: %s' %str(command))
        pub.publish(twist)

def main():
    global pub, home_pub, paused
    paused = False
    home_pub = rospy.Publisher('dog/home', Empty, queue_size=1)
    rospy.init_node('keyboard_input_node', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('dog/homedone', Empty, homedone_callback)

    disp = dispatcher.Dispatcher()
    disp.map("/forward", handle_osc_message, (pub, 'forward'))
    disp.map("/backward", handle_osc_message, (pub, 'backward'))
    disp.map("/left", handle_osc_message, (pub, 'left'))
    disp.map("/right", handle_osc_message, (pub, 'right'))
    disp.map("/turn_left", handle_osc_message, (pub, 'turn_left'))
    disp.map("/turn_right", handle_osc_message, (pub, 'turn_right'))
    disp.map("/stand", handle_osc_message, (pub, 'stand'))
    disp.map("/sit", handle_osc_message, (pub, 'sit'))
    disp.map("/home", handle_home_osc_message)

    server = osc_server.ThreadingOSCUDPServer(('192.168.50.100', 10000), disp)

    keyboard_thread = threading.Thread(target=process_keyboard_input, args=(pub,))
    keyboard_thread.start()

    rospy.loginfo("Serving on %s:%s", server.server_address[0], server.server_address[1])
    server.serve_forever()

global pub,home_pub,paused

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
