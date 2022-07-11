#! /usr/bin/env python3

# import libraries
from matplotlib.pyplot import flag
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import flood_fill_algorithm
import math
import numpy as np


# robot state variables
position_ = Point()
yaw_ = 0
pos_maze_x = -1
pos_maze_y = -1
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
desired_position_.x = -1.35#rospy.get_param(0)
desired_position_.y = 1.23#rospy.get_param(0)
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.02
wall_distance_senstivity = 0.15
wall_distance_senstivity_front_additonal = 0.0
# mapping
maze_size = 16
maze =  [[-1 for i in range(maze_size)] for j in range(maze_size)]
walls = [[0 for i in range(maze_size)] for j in range(maze_size)]

# publishers
pub = None


# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
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
    global region_front, region_fright, region_fleft, region_right, region_left
    regions = {
        'right':  sum(msg.ranges[0:30]) / len(msg.ranges[0:30]),#min(min(msg.ranges[0:90]), 10),
        'fright': min(min(msg.ranges[72:143]), 10),
        'front':  sum(msg.ranges[165:195]) / len(msg.ranges[165:195]),#min(min(msg.ranges[144:215]), 10),
        'fleft':  min(min(msg.ranges[216:287]), 10),
        'left':   sum(msg.ranges[330:360]) / len(msg.ranges[330:360])#min(min(msg.ranges[270:359]), 10),
    }
    # print("Lasers: ", regions['left'],regions['fleft'],regions['front'],regions['fright'],regions['right'])
    region_left = regions['left'] 
    region_fleft = regions['fleft'] 
    region_front = regions['front'] 
    region_fright = regions['fright'] 
    region_right = regions['right'] 

    # take_action(regions)
# Printed phi
def find_orientation():
    '''
    down (-pi/4, pi/4)
    '''
    '''
        3 Up (+ y axis)
        2 Down
        0 Right(+ x axis)
        1 Left
    '''
    global yaw_
    yaw = yaw_
    if -math.pi/4 <= yaw <= math.pi/4:
        return 2#'down'
    elif math.pi/4 <= yaw <= 3*math.pi/4:
        return 0#'right'
    elif -3*math.pi/4 <= yaw <= -math.pi/4:
        return 1#'left'
    elif -2*math.pi <= yaw <= -3*math.pi/4 or 3*math.pi/4 <= yaw <= 2*math.pi:
        return 3#'up'

# set yaw to particular angle 
def set_yaw(desired_yaw):
    global yaw_, yaw_precision_, pub
    err_yaw = normalize_angle(desired_yaw - (yaw_))
    twist_msg = Twist()
    #error of +/- 1 degrees
    if math.fabs(err_yaw) > yaw_precision_/2:
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
    
    if math.fabs(err_yaw) <= yaw_precision_/2:
        twist_msg.angular.z = 0
        change_state(2)
    pub.publish(twist_msg)
    pass

    
# set orientation of the robot to down(0), right(pi), left(-pi), down(2*pi)
def set_orientation(orientation):
    global state_, yaw_
    '''
        0 Right(+ x axis)
        1 Left
        2 Down
        3 Up (+ y axis)
    '''
    if orientation == 2:
        desired_yaw = 0
    elif orientation == 3:
        desired_yaw = -math.pi
    elif orientation == 0:
        desired_yaw = math.pi/2
    elif orientation == 1:
        desired_yaw = -math.pi/2
    print(desired_yaw)
    change_state(0)
    while(state_ != 2):
        set_yaw(desired_yaw)
    else:
        print(yaw_)
        done()


# Printed phi
def update_wall_mapping(x,y):
    global walls, yaw_, region_left, region_fleft, region_front,region_fright, region_right, wall_distance_senstivity
    mouse_orn = find_orientation()
    '''
        0 Right(+ x axis)
        1 Left
        2 Down
        3 Up (+ y axis)

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
    # wall_distance_senstivity = 0.1
    if region_front < wall_distance_senstivity + wall_distance_senstivity_front_additonal:
        state_description += ' F'
        if mouse_orn == 0:
            wallf += 100
        if mouse_orn == 1:
            wallf += 1
        if mouse_orn == 2:
            wallf += 10
        if mouse_orn == 3:
            wallf += 1000
    if region_right < wall_distance_senstivity :
        state_description += ' R'
        if mouse_orn == 0:
            wallr += 10
        if mouse_orn == 1:
            wallr += 1000
        if mouse_orn == 2:
            wallr += 1
        if mouse_orn == 3:
            wallr += 100
    if region_left < wall_distance_senstivity :
        state_description += ' L'
        if mouse_orn == 0:
            walll += 1000
        if mouse_orn == 1:
            walll += 10
        if mouse_orn == 2:
            walll += 100
        if mouse_orn == 3:
            walll += 1
    if wallf != 0:
        if((walls[x][y]//wallf)%10) != 1:
                walls[x][y] += wallf
    if walll != 0:
        if((walls[x][y]//walll)%10) != 1:
                walls[x][y] += walll
    if wallr != 0:
        if((walls[x][y]//wallr)%10) != 1:
                walls[x][y] += wallr
    orien = ["Right","Left","Down","Up"]
    print(orien[mouse_orn],state_description)
    print(state_description, region_left, region_front, region_right)
#function to convert global coordinates to maze coordinates
def convert_to_maze_coordinates(x,y):
    maze_box_size = 0.18
    maze_start = -1.35
    maze_x = int((x - (maze_start)) / maze_box_size)
    maze_y = int((y - (maze_start)) / maze_box_size)
    
    return maze_x, maze_y

#function to convert maze coordinates to global coordinates
def convert_to_global_coordinates(x,y):
    maze_box_size = 0.18
    maze_start = -1.35
    global_x = maze_start + maze_box_size * x
    global_y = maze_start + maze_box_size * y
    
    return global_x, global_y

def change_state(state):
    global state_
    state_ = state
    # print ('State changed to [%s]' % state_)


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
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
    
    pub.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        # print('Yaw error: [%s]' % err_yaw)
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
        # print('Yaw error: [%s]' % err_yaw)
        if state_ != 2:
            change_state(0)
# Printed Soup Song
def done():
    print('Soup Song')
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

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
def move_to_maze_position(x,y):
    global position_, state_
    desired_position_maze = x,y
    desired_position_.x, desired_position_.y  = convert_to_global_coordinates(desired_position_maze[0], desired_position_maze[1])
    state_ = 0
    moveLikeJagger(desired_position_)

#explores the maze
def explore(destination_array_maze_coordinates = [[8,8],[8,7],[7,8],[7,7]]):
    global maze, walls, pos_maze_x, pos_maze_y, state_, desired_position_, maze_size
    
    # destination_array_maze_coordinates = [[8,8],[8,7],[7,8],[7,7]]
    
    #spawn take spawn position and convert to maze coordinates
    pos_maze_x, pos_maze_y = convert_to_maze_coordinates(position_.x, position_.y)
    # move_to_maze_position(0,0)
    # move_to_maze_position(6,0)
    
    #spawn done above
    print("Initialized:: region_left: %s, region_front: %s, region_right: %s" % (region_left, region_front, region_right))
    pos_maze_x, pos_maze_y = 0,15
    update_wall_mapping(pos_maze_x, pos_maze_y)
    #flood fill maze
    flood_fill_algorithm.mod_flood_fill(maze, walls, destination_array_maze_coordinates)
    # print(np.array(maze))
    #for loop
    print("current position: ", pos_maze_x, pos_maze_y)
    print(np.array(walls))
    print(np.array(maze))
    pos_maze_x, pos_maze_y = convert_to_maze_coordinates(position_.x, position_.y)
    
    next_pos_x = 0
    next_pos_y = 16
    pos_maze_x, pos_maze_y = 0,15
    while(maze[pos_maze_x][pos_maze_y] != 0):
        #decision from flood fill maze
        
        #move to decision position
        ####                                     ///uncomment later on
        next_pos_x, next_pos_y = flood_fill_algorithm.determine_next_maze_pos(maze, walls, [pos_maze_x, pos_maze_y])
        print("current position: ", pos_maze_x, pos_maze_y)
        print("next position: ", next_pos_x, next_pos_y)
        move_to_maze_position(next_pos_x, next_pos_y)
        #direction is matrix subtraction nex_pos - pos
        direction = [next_pos_x - pos_maze_x, next_pos_y - pos_maze_y]
        pos_maze_x, pos_maze_y = next_pos_x, next_pos_y
        ####                                     ///uncomment later on
        
        #pseduo decision ///delete this later on
        #########                           ///from here to
        # if next_pos_y > 0:
        # next_pos_y = next_pos_y - 1
        # print(next_pos_x,next_pos_y)
        # move_to_maze_position(next_pos_x,next_pos_y)
        #########                           ///upt0 here
        
        ###correct the orientation before updating walls
        #convert direction to orientation
        orn_wall_map = -1
        if direction[0] == 1:
            orn_wall_map = 0
        if direction[0] == -1:
            orn_wall_map = 1
        if direction[1] == 1:
            orn_wall_map = 3
        if direction[1] == -1:
            orn_wall_map = 2
        set_orientation(orn_wall_map)
        # rospy.sleep(1)
        #update walls
        # pos_maze_x, pos_maze_y = convert_to_maze_coordinates(position_.x, position_.y)
        
        update_wall_mapping(pos_maze_x, pos_maze_y)
        # update_wall_mapping(next_pos_x, next_pos_y)
        
        #update flood fill maze
        flood_fill_algorithm.mod_flood_fill(maze, walls, destination_array_maze_coordinates)
        #delete later one / debug print
        print("current position: ", pos_maze_x, pos_maze_y)
        print(position_)
        print(np.array(walls))
        print(np.array(maze))
        
        pass
    return pos_maze_x, pos_maze_y
    pass

#function to run until odom doesn't return 0
def initialize():
    global position_, region_left, region_fleft, region_front,region_fright, region_right
    flag_position, flag_laser = 0, 0
    print("initializing",end=' ')
    while(1):
        if flag_position == 0:
            if position_.x != 0 and position_.y != 0:
                flag_position = 1
        if flag_laser == 0:
            if (region_right + region_left + region_front) < 30000:
                flag_laser = 1
        if flag_position == 1 and flag_laser == 1:
            break
    print("Initialized, region_left: %s, region_front: %s, region_right: %s" % (region_left, region_front, region_right))
    
def run(end_pos):
    global maze, walls
    flood_fill_algorithm.mod_flood_fill(maze, walls, [end_pos])
    path = flood_fill_algorithm.convert_to_path(maze, walls, end_pos)
    for pos in path:
        x = pos[0]
        y = pos[1]
        move_to_maze_position(x,y)

def main():
    global pub, desired_position_, state_, pos_maze_x, pos_maze_y, maze_size

    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(20)
    #function acting as dummy
    done()
    initialize()
    start_pos = [0,15]
    while not rospy.is_shutdown():
        
        done()
        print("Lets go")
        #function exploring maze
        end_pos = explore()   
        explore([[0,15]])
        
        #run 1
        print("Going for Run 1")
        run(end_pos)
        run(start_pos)
        print()
        
        #run 2
        print("Going for Run 2")
        run(end_pos)
        run(start_pos)
        print()
        
        #run 3
        print("Going for Run 3")
        run(end_pos)
        run(start_pos)
        
        print()
        
        ###     ///debugging set_orientation
        # for i in range(100):
        #     dir = ["RIGHT", "LEFT", "DOWN", "UP"]
        #     print(i,dir[i%4])
        #     set_orientation(i%4)
        #     input("Press Enter to continue...")
        ###     ///debugging set_orientation
        return
        pass

if __name__ == '__main__':
    main()