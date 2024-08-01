#!/usr/bin/env python

import rospy
import actionlib
import yaml
import os

from robot_multipoint.msg import multi_pointAction, multi_pointGoal
from robot_teachpoint.msg import teach_pointAction, teach_pointGoal

link_abs= os.path.abspath(__file__)
link ="{}database/data.yaml".format(link_abs[0:link_abs.index("robot_multipoint")])
run_link ="{}database/run_data.yaml".format(link_abs[0:link_abs.index("robot_multipoint")])

## Send request to teach_point_server
def call_point_server(key,pos,pan,tilt,zoom,rtime,buffer):
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
    rospy.loginfo(result)
## Send request to multi_point_server
def call_task_server(key,name,points,loop):
    client = actionlib.SimpleActionClient(
        'multi_point_server',multi_pointAction)
    client.wait_for_server()

    goal = multi_pointGoal()
    goal.goal_key = key
    goal.goal_name = name
    goal.goal_points = points
    goal.goal_loop = loop

    client.send_goal(goal)
    wait = client.wait_for_result() 
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:         
        rospy.loginfo("TASK DONE !!!!")

    result = client.get_result()
    rospy.loginfo(result)

def main():
    
    print('\n ****** IROPBot ******')
    print('''Choose function
    1 : Move Point
    2 : Save Point
    3 : Save Choose Point
    4 : Delete Point
    5 : Show Points
    6 : MultiPoint
    7 : Save Task
    8 : Delete Task
    9 : Run Task
    10 : Show Task

    .........................
    Press :   ''')
    choose = input()
    if choose == 1 :
        task_choose = 4
        rospy.loginfo('Move_Point')
        print('Point name to move: ')
        pos = raw_input()

        buffer = 1
        call_task_server(task_choose,pos,[],buffer)
        choose = 0

    elif choose == 2:
        rospy.loginfo('Save Point')
        print('Point name to save: ')
        pos = raw_input()

        buffer = 1
        call_point_server(choose,pos,0,0,0,0,buffer)

        choose = 0

    elif choose == 3:
        rospy.loginfo('Save Choose Point')
        print('Point name to save: ')
        pos = raw_input()
        buffer = 1
        print('Please choose point on map to save')
        call_point_server(choose,pos,0,0,0,0,buffer)

        choose = 0

    elif choose == 4:
        rospy.loginfo('Delete Point')
        print('Point name to delete: ')
        pos = raw_input()
        call_point_server(choose,pos,0,0,0,0,0)
        choose = 0

    elif choose == 5:
        rospy.loginfo('Show Points')
        print('on or off: ')
        pos = raw_input()
        call_point_server(choose,pos,0,0,0,0,0)
        choose = 0

    elif choose == 6:
        rospy.loginfo('Multipoint')
        multipoint = []
        print('Enter points name, "e" to end: ')
        while (True):
            k = raw_input()
            if k == 'e':
                break
            multipoint.append(k)
        buffer = 1
        for i in range(0,len(multipoint)):
            print('Running to ', multipoint[i])
            call_point_server(1,multipoint[i],0,0,0,0,buffer)
        rospy.loginfo('MULTIPOINT DONE')
        choose = 0

    elif choose == 7:
        task_choose = 2
        rospy.loginfo('Save Task')
        multipoint = []
        print('Task name to save: ')
        name = raw_input()
        print('Enter points name, "e" to end: ')
        while (True):
            k = raw_input()
            if k == 'e':
                break
            multipoint.append(k)
        print('Loop (1 for on, 0 for off): ')
        l = input()
        call_task_server(task_choose, name, multipoint, l)
        rospy.loginfo('Done SAVE_TASK !!')
        choose = 0

    elif choose == 8:
        task_choose = 3
        rospy.loginfo('Delete Task')
        print('Task name to delete: ')
        name = raw_input()
        call_task_server(task_choose, name, [], 0)
        rospy.loginfo('Done DELETE_TASK !!')
        choose = 0

    elif choose == 9:
        task_choose = 1
        rospy.loginfo('Run Task')
        print('Task name to run: ')
        name = raw_input()
        call_task_server(task_choose, name, [], 0)
        rospy.loginfo('RUN_TASK DONE')
        choose = 0

    elif choose == 10:
        rospy.loginfo('Show Task')
        task_choose = 5
        with open(link, 'r') as stream:
            data = yaml.safe_load(stream)

        for i in range (0,len(data['task'])):
            task_name = list(data['task'])[i]
           
        rospy.loginfo('SHOW_TASK DONE')
        print('Task name to show: ')
        name = raw_input()
        call_task_server(task_choose, name, [], 0)

        choose = 0

    else:
        choose = 0

if __name__ == '__main__':

    rospy.init_node('multipoint_client')
    rospy.loginfo('Beginnn MultiPoint client')
    while(1):
        main()
    
    rospy.spin()
