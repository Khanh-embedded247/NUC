#!/usr/bin/env python

import rospy
import actionlib
import tf
import yaml
import math
import time
import os

from robot_teachpoint.msg import teach_pointAction,teach_pointFeedback, teach_pointResult
#from irop_amcrest_camera.msg import recordAction, recordGoal
#from irop_amcrest_camera.msg import AmcrestControl
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import *
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Int8

#link = '/home/aa/catkin_ws/src/irop-main/irop_ros/database/data.yaml'
#run_link = '/home/aa/catkin_ws/src/irop-main/irop_ros/database/run_data.yaml'
#link = '/home/nhathai/Documents/irop_final/src/IROP_ROS-main/irop_ros/database/data_test/test_data.yaml'
#run_link ='/home/nhathai/Documents/irop_final/src/IROP_ROS-main/irop_ros/database/data_test/test_run_data.yaml'

link_abs= os.path.abspath(__file__)
print(link_abs)
link ="{}database/data.yaml".format(link_abs[0:link_abs.index("robot_teachpoint")])
run_link ="{}database/run_data.yaml".format(link_abs[0:link_abs.index("robot_teachpoint")])

print(run_link)
global choose_point_seq, choose_point, pos_path_global
choose_point_seq = 0
choose_point = PoseStamped()
choose_point.header.seq = 0
pos_path_global = Path()


## Check condition robot STOP
def stop_callback(msg):
    global taskstop
    taskstop = msg.data

## Call cameara server to request
def call_cam_server(key,name,isEmergency):
    # client = actionlib.SimpleActionClient(
    #     'amcrest/record_server',recordAction)
    # client.wait_for_server()

    # goal = recordGoal()
    # goal.goal_key = key
    # goal.goal_name = name
    # goal.goal_emergency = isEmergency

    # client.send_goal(goal)
    # wait = client.wait_for_result() 

    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:         
    #     rospy.loginfo("RECORD DONE !!!!")

    # result = client.get_result()
    # rospy.loginfo(result)
    pass

def global_pth_cb(msg):
    global pos_path_global
    pos_path_global = msg

def map_callback(msg):
    global map
    map = OccupancyGrid()
    map = msg

def savepoint_callback(msg):
    global choose_point 
    choose_point = msg

## Get location of Robot
def get_location():
    
    listener = tf.TransformListener()
    listener.waitForTransform('/map','/base_footprint',rospy.Time(0),rospy.Duration(10000.0))
    (pos,ori) = listener.lookupTransform('/map','/base_footprint',rospy.Time(0))
    robot_x = round((float)(pos[0]),3)
    robot_y = round((float)(pos[1]),3)
    robot_z = round((float)(ori[2]),2)
    robot_w = round((float)(ori[3]),2)
    #print ('Get Location',robot_x, robot_y, robot_z, robot_w)
    return (robot_x,robot_y,robot_z,robot_w)

## Check condition direction of current pose and pose goal 
def check_first():
    global pos_path_global
    pos_path_point = pos_path_global.poses[19]
    # cam_cmd = AmcrestControl()
    # cam_cmd.pan = 180
    # cam_cmd.tilt = 0
    # cam_cmd.zoom = 0
    # pub_cam_cmd.publish(cam_cmd)
    cmd_mux = Int8()
    cmd_mux.data = 1
    pub_cmd_mux.publish(cmd_mux)
    pose_x, pose_y, pose_z, pose_w = get_location()
    theta_goal = math.atan2((pos_path_point.pose.position.y - pose_y ),(pose_x - pos_path_point.pose.position.x)) 
    theta_pose, pitch, roll = tf.transformations.euler_from_quaternion([pose_w, 0, 0, pose_z])
    theta = abs(theta_goal - theta_pose)
    if theta > 2.18 and theta < 3.92:
        while abs(theta_goal - theta_pose) > 0.35 and abs(theta_goal - theta_pose) < 5.93:
            cmd_vel_begin = Twist()
            cmd_vel_begin.angular.z = 0.3
            pub_cmd_begin.publish(cmd_vel_begin)
            pose_x, pose_y, pose_z, pose_w = get_location()
            theta_pose, pitch, roll = tf.transformations.euler_from_quaternion([pose_w, 0, 0, pose_z])
    cmd_mux.data = 0
    pub_cmd_mux.publish(cmd_mux)


## Send pose to Robot move goal pose
def movebase_client(x,y,z,w):
    global pos_path_global
    
    rospy.wait_for_service('/move_base/clear_costmaps')
    clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    clear_costmap()

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #client = actionlib.SimpleActionClient('')
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w 

    client.send_goal(goal)
    now = rospy.Time.now()
    while len(pos_path_global.poses) < 20:
        if rospy.Time.now().secs - now.secs > 2:
            break
    else:
        check_first()
    wait = client.wait_for_result()
    
    
    #rospy.loginfo(wait)

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:         
        rospy.loginfo("MOVE DONE !!!!")
    
    return client.get_state()

## Check condition safety point on the Map to proceed to Save Point
def check_point(x,y):
    xw = (x - map.info.origin.position.x) / map.info.resolution
    yh = (y - map.info.origin.position.y) / map.info.resolution
    i = 0
    j = 0
    for i in range (int(xw) - int(0.6/map.info.resolution), int(xw) + int(0.6/map.info.resolution)):
        for j in range (int(yh) - int(0.6/map.info.resolution), int(yh) + int(0.6/map.info.resolution)):
            point_data = map.data[j * map.info.width + i]
            if point_data == 0:
                p = 0
            else:
                p = 1
                break
        if p == 1:
            break
    if p == 0:
        return True
    else:
        return False

## Save Point Programing
def Save_point(name, pan, tilt, zoom, rtime, buffer):
    rospy.loginfo('Executing SAVE_POINT')
    # Open file data.yaml
    x,y,z,w = get_location()
    if check_point(x, y):
        with open(link, 'r') as stream:
            data = yaml.safe_load(stream)     
        data['point'][name] = {'x' : x ,'y': y,'z':z,'w':w}
        data['point'][name]['cpan'] = pan
        data['point'][name]['ctilt'] = tilt
        data['point'][name]['czoom'] = zoom
        data['point'][name]['rtime'] = rtime
        data['point'][name]['buffer'] = buffer
        show_marker(name,x,y,z,w)
        # Save file data.yaml
        with open(link, 'w') as file:
            yaml.dump(data, file)
        rospy.loginfo('Done SAVE_POINT !!')
    else:
        rospy.loginfo('Bad location, Please choose other point')

## Save Choose Point Programing  
def Save_choose_point(name, pan, tilt, zoom, rtime, buffer):
    global choose_point_seq, choose_point
    rospy.loginfo('Executing SAVE_CHOOSE_POINT')
    # Open file data.yaml
    while choose_point.header.seq == choose_point_seq:
        pass  
    x = choose_point.pose.position.x
    y = choose_point.pose.position.y
    z = choose_point.pose.orientation.z
    w = choose_point.pose.orientation.w
    choose_point_seq = choose_point.header.seq
    if check_point(x, y):
        with open(link, 'r') as stream:
            data = yaml.safe_load(stream)   
        data['point'][name] = {'x' : x ,'y': y,'z':z,'w':w}
        data['point'][name]['cpan'] = pan
        data['point'][name]['ctilt'] = tilt
        data['point'][name]['czoom'] = zoom
        data['point'][name]['rtime'] = rtime
        data['point'][name]['buffer'] = buffer
        show_marker(name,x,y,z,w)
        # Save file data.yaml
        with open(link, 'w') as file:
            yaml.dump(data, file)
        rospy.loginfo('Done SAVE_CHOOSE_POINT !!')
    else:
        rospy.loginfo('Bad location, Please choose other point')

## Delete Point Programing, delete information of Point in database
def Delete_point(name):
    rospy.loginfo('Executing DELETE_POINT')
    # Open file data.yaml
    with open(link , 'r') as stream:
        data = yaml.safe_load(stream)

    del data['point'][name]
    delete_marker(name)

    # Save file data.yaml
    with open(link, 'w') as file:
        yaml.dump(data, file)

    rospy.loginfo('Done DELETE_POINT !!')

## Move Point Programing
def Move(name, rtime, buffer):
    global taskstop
    taskstop = 0
    rospy.loginfo('Executing MOVE')
    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)
    mb_status = movebase_client(data['point'][name]['x'],data['point'][name]['y'],data['point'][name]['z'],data['point'][name]['w'])
    if taskstop == 0:
        if mb_status != 3 :
            mb_status_2 = movebase_client(data['point'][name]['x'],data['point'][name]['y'],data['point'][name]['z'],data['point'][name]['w'])
        else:
            mb_status_2 = 3
        if mb_status_2 == 3:
            if data['point'][name]['buffer'] == 0 and buffer == 0 :
                # cam_cmd = AmcrestControl()
                # cam_cmd.pan = data['point'][name]['cpan']
                # cam_cmd.tilt = data['point'][name]['ctilt']
                # cam_cmd.zoom = data['point'][name]['czoom']
                # pub_cam_cmd.publish(cam_cmd)
                if rtime != 0:
                    call_cam_server(rtime, name, True)
                else:
                    call_cam_server(data['point'][name]['rtime'], name, False)
            return 3
        else: 
            return 2
    else:
        return 2

## Back Home Programing, Robot go to back home positon
def BackHome():
    rospy.loginfo('Executing HOME')
    with open(run_link, 'r') as stream:
        data_run = yaml.safe_load(stream)
    mb_status = movebase_client(data_run['startup']['home']['x'],data_run['startup']['home']['y'],data_run['startup']['home']['z'],data_run['startup']['home']['w'])
    if mb_status == 2:
        mb_status_2 = movebase_client(data_run['startup']['home']['x'],data_run['startup']['home']['y'],data_run['startup']['home']['z'],data_run['startup']['home']['w'])
    else:
        mb_status_2 = 3
    if mb_status_2 == 3:
        return 3
    else: 
        return 2

## Show Points Programing
def show_all(name):
    rospy.loginfo('Executing SHOW_POINTS')

    with open(link , 'r') as stream:
        data = yaml.safe_load(stream)

    if name == 'on':
        for i in range (0,len(data['point'])):
            data_name = list(data['point'])[i]
            show_marker(data_name,data['point'][data_name]['x'],data['point'][data_name]['y'],data['point'][data_name]['z'],data['point'][data_name]['w'])
    if name == 'off':
        for i in range (0,len(data['point'])):
            data_name = list(data['point'])[i]
            delete_marker(data_name)
    

## show_marker Programing, display Point on the map
def show_marker(name,x,y,z,w):
    
    marker_ = MarkerArray()

    marker_text = Marker()
    #marker_text.header.seq +=1
    marker_text.header.frame_id ="/map"
    marker_text.header.stamp = rospy.Time.now()
    marker_text.type = marker_text.TEXT_VIEW_FACING
    marker_text.action = marker_text.ADD
    marker_text.ns = name
    #marker_text.id= id
    marker_text.pose.position.x = x + 0.3
    marker_text.pose.position.y = y
    marker_text.pose.position.z = 0.01

    marker_text.pose.orientation.x = 0
    marker_text.pose.orientation.y = 0
    marker_text.pose.orientation.z = 0
    marker_text.pose.orientation.w = 1
  
    #marker_text.scale.x = 0.1
    #marker_text.scale.y = 0.1
    marker_text.scale.z = 0.5

    marker_text.color.a = 1.0
    marker_text.color.r = 1.0
    marker_text.color.g = 0.0
    marker_text.color.b = 0.0

    marker_text.text = name

    marker_.markers.append(marker_text)

    marker_point = Marker()
    #marker_text.header.seq +=1
    marker_point.header.frame_id ="/map"
    marker_point.header.stamp = rospy.Time.now()
    marker_point.type = marker_point.ARROW
    marker_point.action = marker_point.ADD
    marker_point.ns = name + "_point"
    marker_point.pose.position.x = x
    marker_point.pose.position.y = y
    marker_point.pose.position.z = 0.01

    marker_point.pose.orientation.x = 0
    marker_point.pose.orientation.y = 0
    marker_point.pose.orientation.z = z
    marker_point.pose.orientation.w = w
  
    marker_point.scale.x = 0.2
    marker_point.scale.y = 0.1
    marker_point.scale.z = 0.9

    marker_point.color.a = 1.0
    marker_point.color.r = 1.0
    marker_point.color.g = 0.0
    marker_point.color.b = 0.0

    marker_.markers.append(marker_point)

    #rospy.loginfo(marker_)
    pub_marker.publish(marker_)

## Delete_marker Programing, delete markers on the map 
def delete_marker(name):
    
    marker_ = MarkerArray()

    marker_text = Marker()
    #marker_text.header.seq +=1
    marker_text.header.frame_id ="/map"
    marker_text.header.stamp = rospy.Time.now()
    marker_text.type = marker_text.TEXT_VIEW_FACING
    marker_text.action = marker_text.DELETE
    marker_text.ns = name
    #marker_text.id= id

    marker_.markers.append(marker_text)

    marker_point = Marker()
    #marker_text.header.seq +=1
    marker_point.header.frame_id ="/map"
    marker_point.header.stamp = rospy.Time.now()
    marker_point.type = marker_text.ARROW
    marker_point.action = marker_text.DELETE
    marker_point.ns = name + "_point"
    #marker_point.id= id

    marker_.markers.append(marker_point)

    #rospy.loginfo(marker_)
    pub_marker.publish(marker_)


## Main Programing
class Action_Server():
    def __init__(self):
        self.teach_server = actionlib.SimpleActionServer(
                    'teach_point_server',teach_pointAction,execute_cb=self.execute_cb,auto_start= False
                    )
        self.teach_server.start()
    def execute_cb(self,goal):

        feedback = teach_pointFeedback()
        result_a = teach_pointResult()

        feedback_str = ''
        result_str = ''
        rate = rospy.Rate(20)

        ## Condition to Move Point
        if goal.goal_key == 1:
            rospy.loginfo('Move_Point')
            #rospy.loginfo(goal.goal_pos)

            feedback_str ='Movingggg'
            feedback.feedback_key = feedback_str

            result_str = 'MOVE POINT DONE !!!'
            result_a.result_key =result_str

            self.teach_server.publish_feedback(feedback)

            result_a.result_status = Move(goal.goal_pos, goal.goal_rtime , goal.goal_buffer)

        ## Condition to Save Point, Save the current possiton of Robot
        elif goal.goal_key == 2:
            rospy.loginfo('Save Point')
            #rospy.loginfo(goal.goal_pos)

            feedback_str ='Savingg'
            feedback.feedback_key = feedback_str

            result_str = 'SAVE POINT DONE !!!'
            result_a.result_key = result_str
            result_a.result_status = 3

            self.teach_server.publish_feedback(feedback)

            Save_point(goal.goal_pos, goal.goal_pan, goal.goal_tilt, goal.goal_zoom, goal.goal_rtime, goal.goal_buffer)

        ## Condition to Save Choose Point
        elif goal.goal_key == 3:
            rospy.loginfo('Save Choose Point')
            #rospy.loginfo(goal.goal_pos)

            feedback_str ='Savingg'
            feedback.feedback_key = feedback_str

            result_str = 'SAVE CHOOSE POINT DONE !!!'
            result_a.result_key = result_str
            result_a.result_status = 3

            self.teach_server.publish_feedback(feedback)

            Save_choose_point(goal.goal_pos, goal.goal_pan, goal.goal_tilt, goal.goal_zoom, goal.goal_rtime, goal.goal_buffer)

        ## Condition to Delete Point
        elif goal.goal_key == 4:
            rospy.loginfo('Delete Point')
            #rospy.loginfo(goal.goal_pos)
            feedback_str ='Deleteeee'
            feedback.feedback_key = feedback_str

            result_str = 'DELETE POINT DONE !!!'  
            result_a.result_key = result_str
            result_a.result_status = 3

            self.teach_server.publish_feedback(feedback)
            Delete_point(goal.goal_pos)   

        ## Condition to Show Points, Show markers of Point on screen
        elif goal.goal_key == 5:
            rospy.loginfo('Show Points')
            #rospy.loginfo(goal.goal_pos)
            feedback_str ='Show'
            feedback.feedback_key = feedback_str

            result_str = 'SHOW POINTS DONE !!!'  
            result_a.result_key = result_str
            result_a.result_status = 3

            self.teach_server.publish_feedback(feedback)
            show_all(goal.goal_pos)   

        ## Condition to Back Home, Robot go to back home position    
        elif goal.goal_key == 6:
            rospy.loginfo('Back Home')
            #rospy.loginfo(goal.goal_pos)

            feedback_str ='Homming'
            feedback.feedback_key = feedback_str

            result_str = 'BACK HOME DONE !!!'
            result_a.result_key =result_str

            self.teach_server.publish_feedback(feedback)

            result_a.result_status = BackHome()
        
        ## Condition is wrong
        else:
            rospy.loginfo('Wrong input')
            #rospy.loginfo(goal.goal_pos)
            feedback_str ='Wrong input'
            feedback.feedback_key = feedback_str

            result_str = 'PLEASE ENTER AGAIN'  
            result_a.result_key = result_str
            result_a.result_status = 1

            self.teach_server.publish_feedback(feedback)       
               
        self.teach_server.set_succeeded(result_a)


if __name__ == "__main__":

    rospy.init_node('teach_point_server')
    pub_marker = rospy.Publisher('points_visualization', MarkerArray, queue_size=100)
    pub_cmd_mux = rospy.Publisher('cmd_mux_choose', Int8, queue_size=100)
    pub_cmd_begin = rospy.Publisher('cmd_vel_begin', Twist, queue_size=100)
    #pub_cam_cmd = rospy.Publisher('amcrest/control', AmcrestControl, queue_size=100)
    rospy.Subscriber("multipoint_server/stop", Int8, stop_callback)
    rospy.Subscriber("save_point", PoseStamped, savepoint_callback)
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.Subscriber('move_base/NavfnROS/plan', Path, global_pth_cb)
    rospy.loginfo('Begin Teach Point Server')
    
    # Main programing
    s = Action_Server()
    rospy.spin()
