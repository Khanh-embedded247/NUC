#!/usr/bin/env python


import rospy
import actionlib
import yaml
import os

from robot_multipoint.msg import multi_pointAction, multi_pointFeedback, multi_pointResult
from robot_teachpoint.msg import teach_pointAction, teach_pointGoal, teach_pointResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry,Path

link_abs= os.path.abspath(__file__)
link ="{}database/data.yaml".format(link_abs[0:link_abs.index("robot_multipoint")])
run_link ="{}database/run_data.yaml".format(link_abs[0:link_abs.index("robot_multipoint")])

print(run_link)
def stop_callback(msg):
    global taskstop
    taskstop = msg.data
    rospy.loginfo(taskstop)


## Check multipoint client 
def call_server(key,pos,pan,tilt,zoom,rtime,buffer):
    client = actionlib.SimpleActionClient(
        'teach_point_server',teach_pointAction)
    client.wait_for_server()

    goal = teach_pointGoal()
    goal.goal_key = key
    goal.goal_pos = pos
    goal.goal_pan = pan
    goal.goal_tilt = tilt
    goal.goal_zoom = zoom
    goal.goal_rtime = rtime
    goal.goal_buffer = buffer

    client.send_goal(goal)
    wait = client.wait_for_result() 
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:         
        rospy.loginfo("MOVE DONE !!!!")

    result = client.get_result()
    
    while result is None:
        result = client.get_result()
    
    rospy.loginfo(result.result_key)
    return result.result_status
    

# Save Task Programing
def save_task(name, points, loop, wait):
    rospy.loginfo('Executing SAVE_TASK')
    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)
    data['task'][name] = {}
    data['task'][name]['points'] = points
    data['task'][name]['loop'] = loop
    data['task'][name]['wait'] = wait
    with open(link, 'w') as file:
        yaml.dump(data, file)    
    rospy.loginfo('Done SAVE_TASK !!')

## Delete Task Programing
def delete_task(name):
    rospy.loginfo('Executing DELETE_TASK')
    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)
    del data['task'][name]
    with open(link, 'w') as file:
        yaml.dump(data, file)
    rospy.loginfo('Done DELETE_TASK !!')


## Run Task Programing, robot move to each point in Task
def run_task(name, index):
    global taskstop
    taskstop = 0
    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)
    multipoint = data['task'][name]['points']
    with open(run_link, 'r') as stream:
        data_run = yaml.safe_load(stream)
    data_run['running']['lastest_task'] = name
    l = 0
    loop = data['task'][name]['loop']
    # while (True):
    show_all(multipoint)
    if l == 0:
        i = index
    else:
        i = 0
    if i >= len(multipoint):
        task_status = 3
    while i < len(multipoint):
        rospy.loginfo("Running to " + str(multipoint[i]))
        data_run['running']['lastest_task_index'] = i
        with open(run_link, 'w') as file:
            yaml.dump(data_run, file)
        task_status = call_server(1,multipoint[i],0,0,0,0,0)
        if taskstop == 1 or task_status == 2:
            break
        elif taskstop == 2:
            rospy.loginfo(taskstop)
            while True:
                if taskstop == 0:
                    break
        else:
            change_color(multipoint,i)
            i = i+1
            # now = rospy.Time.now()
            # if time_wait > 0:
            #     print "wait " + str(time_wait) + " sec"
            #     while ((rospy.Time.now().secs - now.secs) < time_wait) :
            #         if taskstop == 1:
            #             break
            #         elif taskstop == 2:
            #             now = rospy.Time.now()
    delete_all(multipoint)
        # if (loop == 1) and (taskstop == 0) and (task_status !=2):
        #     l = 1
        # else:
        #     break
    if task_status == 3 and taskstop == 0:
        data_run['running']['lastest_task_index'] = 0
        with open(run_link, 'w') as file:
            yaml.dump(data_run, file)
    return task_status

## Show Points Programing
def show_all(points):
    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)
    for i in range (0,len(points)):
        point_name = points[i]
        show_marker(str(i+1),data['point'][point_name]['x'],data['point'][point_name]['y'])

## Delete all markers
def delete_all(points):
    for i in range (0,len(points)):
        delete_marker(str(i+1))


## Change color Markers
def change_color(points, point_i):
    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)
    point_name = points[point_i]

    marker_ = MarkerArray()

    marker_text = Marker()
    #marker_text.header.seq +=1
    marker_text.header.frame_id ="/map"
    marker_text.header.stamp = rospy.Time.now()
    marker_text.type = marker_text.TEXT_VIEW_FACING
    marker_text.action = marker_text.ADD
    marker_text.ns = str(point_i+1)
    #marker_text.id= id
    marker_text.pose.position.x = data['point'][point_name]['x'] - 0.3
    marker_text.pose.position.y = data['point'][point_name]['y']
    marker_text.pose.position.z = 0.01

    marker_text.pose.orientation.x = 0
    marker_text.pose.orientation.y = 0
    marker_text.pose.orientation.z = 0
    marker_text.pose.orientation.w = 1
  
    #marker_text.scale.x = 0.1
    #marker_text.scale.y = 0.1
    marker_text.scale.z = 0.5

    marker_text.color.a = 1.0
    marker_text.color.r = 0.5
    marker_text.color.g = 0.5
    marker_text.color.b = 0.5

    marker_text.text = str(point_i+1)

    marker_.markers.append(marker_text)

    marker_point = Marker()
    #marker_text.header.seq +=1
    marker_point.header.frame_id ="/map"
    marker_point.header.stamp = rospy.Time.now()
    marker_point.type = marker_text.CYLINDER
    marker_point.action = marker_text.ADD
    marker_point.ns = str(point_i+1) + "_point"
    #marker_point.id= id
    marker_point.pose.position.x = data['point'][point_name]['x']
    marker_point.pose.position.y = data['point'][point_name]['y']
    marker_point.pose.position.z = 0.01

    marker_point.pose.orientation.x = 0
    marker_point.pose.orientation.y = 0
    marker_point.pose.orientation.z = 0
    marker_point.pose.orientation.w = 1
  
    marker_point.scale.x = 0.2
    marker_point.scale.y = 0.2
    marker_point.scale.z = 1.0

    marker_point.color.a = 1.0
    marker_point.color.r = 0.5
    marker_point.color.g = 0.5
    marker_point.color.b = 0.5

    marker_.markers.append(marker_point)

    #rospy.loginfo(marker_)
    pub_marker.publish(marker_)

## show_marker Programing, display Point on the map
def show_marker(name,x,y):
    
    marker_ = MarkerArray()

    marker_text = Marker()
    #marker_text.header.seq +=1
    marker_text.header.frame_id ="/map"
    marker_text.header.stamp = rospy.Time.now()
    marker_text.type = marker_text.TEXT_VIEW_FACING
    marker_text.action = marker_text.ADD
    marker_text.ns = name
    #marker_text.id= id
    marker_text.pose.position.x = x - 0.3
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
    marker_text.color.r = 0.0
    marker_text.color.g = 1.0
    marker_text.color.b = 0.0

    marker_text.text = name

    marker_.markers.append(marker_text)

    marker_point = Marker()
    #marker_text.header.seq +=1
    marker_point.header.frame_id ="/map"
    marker_point.header.stamp = rospy.Time.now()
    marker_point.type = marker_text.CYLINDER
    marker_point.action = marker_text.ADD
    marker_point.ns = name + "_point"
    #marker_point.id= id
    marker_point.pose.position.x = x
    marker_point.pose.position.y = y
    marker_point.pose.position.z = 0.01

    marker_point.pose.orientation.x = 0
    marker_point.pose.orientation.y = 0
    marker_point.pose.orientation.z = 0
    marker_point.pose.orientation.w = 1
  
    marker_point.scale.x = 0.2
    marker_point.scale.y = 0.2
    marker_point.scale.z = 1.0

    marker_point.color.a = 1.0
    marker_point.color.r = 0.0
    marker_point.color.g = 1.0
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
    marker_point.type = marker_text.CYLINDER
    marker_point.action = marker_text.DELETE
    marker_point.ns = name + "_point"
    #marker_point.id= id

    marker_.markers.append(marker_point)

    #rospy.loginfo(marker_)
    pub_marker.publish(marker_)

def show_path(name):

    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)
    multipoint = data['task'][name]['points']

    msg_path = Path()

    msg_path.header.frame_id = "map"
    msg_path.header.stamp = rospy.Time.now()

    with open(link, 'r') as stream:
        data = yaml.safe_load(stream)

    for i in range (0,len(multipoint)):
        point_name = multipoint[i]
        show_marker(str(i+1),data['point'][point_name]['x'],data['point'][point_name]['y'])
        
        pose = PoseStamped()
        pose.pose.position.x = data['point'][point_name]['x']
        pose.pose.position.y = data['point'][point_name]['y']
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = data['point'][point_name]['z']
        pose.pose.orientation.w = data['point'][point_name]['w']
        msg_path.poses.append(pose)
    
        pub_path.publish(msg_path)

## Main Programing
class Action_Server():
    def __init__(self):
        self.multi_server = actionlib.SimpleActionServer(
                    'multi_point_server',multi_pointAction,execute_cb=self.execute_cb,auto_start= False
                    )
        self.multi_server.start()
    def execute_cb(self,goal):
    
        feedback = multi_pointFeedback()
        result_a = multi_pointResult()

        feedback_str = ''
        result_str = ''
        rate = rospy.Rate(20)

        # Condition to Run Task, Robot move to point in Tassk
        if goal.goal_key == 1:
            rospy.loginfo('Run task')
            #rospy.loginfo(goal.goal_pos)

            feedback_str ='running'
            feedback.feedback_key = feedback_str

            result_str = 'RUN TASK DONE !!!'
            result_a.result_key =result_str


            result_a.result_status = run_task(goal.goal_name, goal.goal_loop)


            self.multi_server.publish_feedback(feedback)

        # Condition to Save Task 
        elif goal.goal_key == 2:
            rospy.loginfo('Save task')
            #rospy.loginfo(goal.goal_pos)

            feedback_str ='Saving'
            feedback.feedback_key = feedback_str

            result_str = 'SAVE TASK DONE !!!'
            result_a.result_key = result_str
            result_a.result_status = 3

            self.multi_server.publish_feedback(feedback)

            save_task(goal.goal_name, goal.goal_points, goal.goal_loop, goal.goal_wait)
        
        # Condition to Delete Task
        elif goal.goal_key == 3:
            rospy.loginfo('Delete task')
            #rospy.loginfo(goal.goal_pos)
            feedback_str ='Delete'
            feedback.feedback_key = feedback_str

            result_str = 'DELETE TASK DONE !!!'  
            result_a.result_key = result_str
            result_a.result_status = 3

            self.multi_server.publish_feedback(feedback)
            delete_task(goal.goal_name)
        
        # Condition to Move Point
        elif goal.goal_key == 4:
            rospy.loginfo('Move_Point')
            feedback_str ='moving'
            feedback.feedback_key = feedback_str

            result_str = 'MOVE POINT DONE !!!'
            result_a.result_key =result_str

            self.multi_server.publish_feedback(feedback)
            result_a.result_status = call_server(1,goal.goal_name,0,0,0,0,goal.goal_loop)
        elif  goal.goal_key == 5:
            rospy.loginfo('Show Task')

            feedback_str ='ShowShow'
            feedback.feedback_key = feedback_str

            result_str = 'Show task DONE !!!'
            result_a.result_key = result_str
            result_a.result_status = 3

            self.multi_server.publish_feedback(feedback)

            show_path(goal.goal_name)

        # Conditoin is Wrong
        else:
            rospy.loginfo('Wrong input')
            #rospy.loginfo(goal.goal_pos)
            feedback_str = 'Wrong input'
            feedback.feedback_key = feedback_str

            result_str = 'PLEASE ENTER AGAIN'  
            result_a.result_key = result_str
            result_a.result_status = 1

            self.multi_server.publish_feedback(feedback)   
               
        self.multi_server.set_succeeded(result_a)

if __name__ == "__main__":
    
    rospy.init_node('multi_point_server')
    pub_marker = rospy.Publisher('task_visualization', MarkerArray, queue_size=100)
    pub_path = rospy.Publisher('path_of_robot', Path, queue_size=100)
    rospy.Subscriber("multipoint_server/stop", Int8, stop_callback)
    rospy.loginfo('Begin Multi Point Server')
    
    # Main Programing
    s = Action_Server()
    rospy.spin()

