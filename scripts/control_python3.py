#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
#import cool stuff
import masti
import flood_fill_algorithm


import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
pos_x = -1
pos_y = -1
# laser variables
region_front = 10000
region_fright = 10000
region_fleft = 10000
region_right = 10000
region_left = 10000
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = -1.35536242235#rospy.get_param(0)
desired_position_.y = 0#rospy.get_param(0)
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.03
# mapping
maze_size = 16
maze =  [[-1 for i in range(maze_size)] for j in range(maze_size)]
walls = [[0 for i in range(maze_size)] for j in range(maze_size)]

# publishers
pub = None


# service callbacks
# def go_to_point_switch(req):
#     global active_
#     active_ = req.data
#     res = SetBoolResponse()
#     res.success = True
#     res.message = 'Done!'
#     return res

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    # print('position_: ', position_)
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

# Printed Lasers
def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:90]), 10),
        'fright': min(min(msg.ranges[72:143]), 10),
        'front':  min(min(msg.ranges[144:215]), 10),
        'fleft':  min(min(msg.ranges[216:287]), 10),
        'left':   min(min(msg.ranges[270:359]), 10),
    }
    # print()
    # print("Lasers: ", regions['left'],regions['fleft'],regions['front'],regions['fright'],regions['right'])
    region_left = regions['left'] 
    region_fleft = regions['fleft'] 
    region_front = regions['front'] 
    region_fright = regions['fright'] 
    region_right = regions['right'] 
        # print()
    # take_action(regions)
# Printed phi
def find_orientation():
    '''
    down (-pi/4, pi/4)
    '''
    global yaw_
    yaw = yaw_
    if -math.pi/4 <= yaw <= math.pi/4:
        return 1#'down'
    elif math.pi/4 <= yaw <= 3*math.pi/4:
        return 2#'right'
    elif -3*math.pi/4 <= yaw <= -math.pi/4:
        return 3#'left'
    elif -2*math.pi <= yaw <= -3*math.pi/4 or 3*math.pi/4 <= yaw <= 2*math.pi:
        return 0#'up'
# Printed phi
def update_wall_mapping(x,y):
    global walls, yaw_, region_left, region_fleft, region_front,region_fright, region_right
    mouse_orn = find_orientation()
    '''
        0 Up (+ y axis)
        1 Down
        2 Right(+ x axis)
        3 Left

        w/ reference to +y axis
        -left wall 1
        -down wall 10
        -right wall 100
        -up wall 1000
    '''
    wallf = 0
    walll = 0
    wallr = 0
    state_description = ''
    if region_front < 0.11:
        state_description += ' F'
        if mouse_orn == 0:
            wallf += 1000
        if mouse_orn == 1:
            wallf += 10
        if mouse_orn == 2:
            wallf += 100
        if mouse_orn == 3:
            wallf += 1
    if region_right < 0.11:
        state_description += ' R'
        if mouse_orn == 0:
            wallr += 100
        if mouse_orn == 1:
            wallr += 1
        if mouse_orn == 2:
            wallr += 10
        if mouse_orn == 3:
            wallr += 1000
    if region_left < 0.11:
        state_description += ' L'
        if mouse_orn == 0:
            walll += 1
        if mouse_orn == 1:
            walll += 100
        if mouse_orn == 2:
            walll += 1000
        if mouse_orn == 3:
            walll += 10
    if wallf != 0:
        if((walls[x][y]//wallf)%10) != 1:
                walls[x][y] += wallf
    if walll != 0:
        if((walls[x][y]//walll)%10) != 1:
                walls[x][y] += walll
    if wallr != 0:
        if((walls[x][y]//wallr)%10) != 1:
                walls[x][y] += wallr
# Printed Stste changed to
def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
# Printed Walla Walla
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - (yaw_ - math.pi/2))
    
    # rospy.loginfo(err_yaw)
    # rospy.loginfo(desired_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        # twist_msg.angular.z = 0
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)
    # print('Walla Walla')
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print('Yaw error: [%s]' % err_yaw)
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - (yaw_ - math.pi/2)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = 0
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print('Yaw error: [%s]' % err_yaw)
        change_state(0)
# Printed Soup Song
def done():
    print('Soup Song')
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

# des_x,des_y -> Block no. 
def n_block_coordinate(des_x,des_y):
    desired_position_.x = 0.17978 * (des_x-8)+0.08989#-0.072492 #-1.3
    des_y+=1
    desired_position_.y = -0.17978 * (des_y-8)+0.08989 #

def moveLikeJagger(des_pos):
    print('Desired position MLJ: [%s, %s]' % (des_pos.x, des_pos.y))
    while(state_ != 2): #Here was the logical error - replaced if with while
        # State_Msg = 'State: [%s]' % state_
        # rospy.loginfo(State_Msg)
        if state_ == 0:
            fix_yaw(des_pos)
        elif state_ == 1:
            go_straight_ahead(des_pos)
        else:
            rospy.logerr('Unknown state!')
    else:
        done()
# Printed Thnda, hola
def initial_position():
    global  maze_size
    '''
        find quadrant of a point
    '''
    print("Thnda", position_)
    x = position_.x
    y = position_.y
    size = maze_size
    quad = -1
    posi_x = -1
    posi_y = -1
    if x > 0 and y > 0:
        quad = 1
    elif x < 0 and y > 0:
        quad = 2
    elif x < 0 and y < 0:
        quad = 3
    elif x > 0 and y < 0:
        quad = 4

    if quad == 1:
        posi_x = size - 1
        posi_y = 0
    if quad == 2:
        posi_x = 0
        posi_y = 0
    if quad == 3:
        posi_x = 0
        posi_y = size - 1
    if quad == 4:
        posi_x = size - 1
        posi_y = size - 1
    print('Hola', quad, x, y)
    return posi_x, posi_y

def set_next_des_pos():
    '''
        w/ reference to +y axis
        -left wall 1
        -down wall 10
        -right wall 100
        -up wall 1000
    '''
    global maze, pos_x, pos_y, maze_size, walls, desired_position_
    x = pos_x
    y = pos_y
    print('wow', x, y)
    matrix = maze
    if x + 1 < maze_size:
        if (walls[x][y]//1000)%10 == 0:
            if maze[x + 1][y] < min(matrix[x][y-1], matrix[x][y+1], matrix[x-1][y]):
                desired_position_.x = x
                desired_position_.y = y
    if x - 1 > 0:
        if (walls[x][y]//10)%10 == 0:
            if maze[x - 1][y] < min(matrix[x][y-1], matrix[x][y+1], matrix[x+1][y]):
                desired_position_.x = x
                desired_position_.y = y
    if y + 1 < maze_size:
        if (walls[x][y]//100)%10 == 0:
            if maze[x][y + 1] < min(matrix[x][y-1], matrix[x-1][y], matrix[x+1][y]):
                desired_position_.x = x
                desired_position_.y = y
    if y > 0:
        if (walls[x][y]//1)%10 == 0:
            if maze[x][y - 1] < min(matrix[x][y+1], matrix[x-1][y], matrix[x+1][y]):
                desired_position_.x = x
                desired_position_.y = y
                


def main():
    global pub, active_, pos_x, pos_y

    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(20)
    pos_x, pos_y  = initial_position() # current position
    pos_x, pos_y  = 0,0 #-1.351300, 1.233343 # current position
    print('Initial position: ', pos_x, pos_y)
    while not rospy.is_shutdown():
        update_wall_mapping(pos_x,pos_y)
        flood_fill_algorithm.mod_flood_fill(maze, walls, 8, 8)
        # rospy.loginfo(maze)
        set_next_des_pos()
        moveLikeJagger(desired_position_)
        pos_x = desired_position_.x
        pos_y = desired_position_.y
        print('Khatam')
        # masti.boloNamaste()
        # b = find_orientation()
        # print(b)
    
        
        rate.sleep()

if __name__ == '__main__':
    # n_block_coordinate(4, 6)
    main()
