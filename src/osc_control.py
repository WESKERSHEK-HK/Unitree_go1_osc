#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Empty, Float64
import sys
import tty
import termios
from pythonosc import dispatcher, osc_server, udp_client
import threading

def cmd_vel_callback(msg):
    global client_ip
    osc_client = udp_client.SimpleUDPClient(client_ip, 20000)
    osc_client.send_message("/cmd_vel", [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z])

def handle_position_osc_message(unused_addr, x, y, z):
    global position_pub
    position = Point(x, y, z)
    position_pub.publish(position)

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
                twist.linear.x = 0.2
            elif ch == 's':
                twist.linear.x = -0.2
            elif ch == 'a':
                twist.linear.y = 0.2
            elif ch == 'd':
                twist.linear.y = -0.2
            elif ch == 'j':
                twist.angular.z = 0.2
            elif ch == 'l':
                twist.angular.z = -0.2
            elif ch == 'm':
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0

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
        command = args[0]
        twist = Twist()

        if command == 'forward':
            twist.linear.x = 0.2
        elif command == 'backward':
            twist.linear.x = -0.2
        elif command == 'left':
            twist.linear.y = 0.2
        elif command == 'right':
            twist.linear.y = -0.2
        elif command == 'turn_left':
            twist.angular.z = 0.2
        elif command == 'turn_right':
            twist.angular.z = -0.2
        elif command == 'stop':
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        
        print('Received osc command: %s' %str(command))
        print(twist)
        pub.publish(twist)

def yaw_data_callback(msg):
    global client_ip
    osc_client = udp_client.SimpleUDPClient(client_ip, 20000)
    osc_client.send_message("/yaw_data", msg.data)

def main():
    global pub, home_pub, paused, stop_pub, position_pub, client_ip
    paused = False
    client_ip = "192.168.3.139"
    device_ip = "192.168.3.124"

    rospy.init_node('keyboard_input_node', anonymous=True)

    home_pub = rospy.Publisher('dog/home', Empty, queue_size=1)
    stop_pub = rospy.Publisher('dog/stop', Empty, queue_size=1)
    position_pub = rospy.Publisher('dog/position', Point, queue_size=1)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('yaw_data', Float64, yaw_data_callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('dog/homedone', Empty, homedone_callback)


    disp = dispatcher.Dispatcher()
    disp.map("/forward", handle_osc_message, ('forward'))
    disp.map("/backward", handle_osc_message, ('backward'))
    disp.map("/left", handle_osc_message, ('left'))
    disp.map("/right", handle_osc_message, ('right'))
    disp.map("/turn_left", handle_osc_message, ('turn_left'))
    disp.map("/turn_right", handle_osc_message, ('turn_right'))
    disp.map("/stop", handle_osc_message, ('stop'))
    disp.map("/home", handle_home_osc_message)
    disp.map("/position", handle_position_osc_message)

    server = osc_server.ThreadingOSCUDPServer((device_ip, 10000), disp)

    keyboard_thread = threading.Thread(target=process_keyboard_input, args=(pub,))
    keyboard_thread.start()

    rospy.loginfo("Serving on %s:%s", server.server_address[0], server.server_address[1])
    server.serve_forever()

global pub,home_pub,paused,client_ip

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
