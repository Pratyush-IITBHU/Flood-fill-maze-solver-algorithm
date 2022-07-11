#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import sys, select, termios, tty
settings = termios.tcgetattr(sys.stdin)



x_dist = 0
y_dist = 0

def clbk_odom(msg):
    global x_dist
    global y_dist
    global yaw_
    global position_
    # position
    position_ = msg.pose.pose.position
    # gives x and y distance of the bot
    x_dist = position_.x
    y_dist = position_.y
    
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    yaw_ = euler[2]
    # print(x_dist, y_dist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
### FOR TESTING PURPOSES OF  KEYBOARD
# key = '1'
# while(key!='0'):
#     settings = termios.tcgetattr(sys.stdin)
#     key = getKey(2)
#     if key in ('a','s','w','d'):
#         print("Namaste",key)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w
   a    s    d

CTRL-C or q to quit
"""

def main():
    max_velocity = 0.4
    max_ang_velocity = 0.4
    print(msg)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)  
    rospy.init_node('cmd_robot', anonymous=True)
    rate = rospy.Rate(50) # 40hz
    settings = termios.tcgetattr(sys.stdin)

    key = 'p'

    while not rospy.is_shutdown() and key != 'q':
        msg1 = Twist()
        #positive speed_z value represents clockwise angular velocity of the bot and positive speed_x value represents forward linear velocity of the robot
        
        key = getKey(0.1)
        if key in ('a','s','w','d'):
            print("Namaste",key,yaw_, position_)
            if key == 'w':
                speed_x = max_velocity
            if key == 's':
                speed_x = -max_velocity
            if key == 'a':
                speed_z = max_ang_velocity
            if key == 'd':
                speed_z = -max_ang_velocity
        else:
            speed_z = 0
            speed_x = 0.0

        msg1.linear.x = speed_x
        msg1.angular.z = speed_z
        pub.publish(msg1)
        rate.sleep()

if __name__ == '__main__':
    main()