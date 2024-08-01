#!/usr/bin/env python

import rospy
import actionlib

from robot_teachpoint.msg import teach_pointAction, teach_pointGoal

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
    rospy.loginfo(result)

def main():

    print('\n ****** IROPBot ******')
    print('''Choose function
    1 : Move Point
    2 : Save Point
    3 : Save Choose Point
    4 : Delete Point
    5 : Show Points
    6 : Back Home

    .........................
    Press :   ''')
    choose = input()
    if choose == 1 :
        rospy.loginfo('Move_Point')
        print('Point name to move: ')
        pos = raw_input()
        print('run as buffer point (1 for true, 0 for false): ')
        buffer = input()
        call_server(choose,pos,0,0,0,0,buffer)
        choose = 0

    elif choose == 2:
        rospy.loginfo('Save Point')
        print('Point name to save: ')
        pos = raw_input()
        print('is this buffer point (1 for true, 0 for false): ')
        buffer = input()
        if buffer != 0:
            call_server(choose,pos,0,0,0,0,buffer)
        else:
            print('camera pan: ')
            pan = input()
            print('camera_tilt: ')
            tilt = input()
            print('camera_zoom: ')
            zoom = input()
            print('record time (in second): ')
            rtime = input()
            call_server(choose,pos,pan,tilt,zoom,rtime,buffer)
        choose = 0

    elif choose == 3:
        rospy.loginfo('Save Choose Point')
        print('Point name to save: ')
        pos = raw_input()
        print('is this buffer point (1 for true, 0 for false): ')
        buffer = input()
        if buffer != 0:
            print('Please choose point on map to save')
            call_server(choose,pos,0,0,0,0,buffer)
        else:
            print('camera pan: ')
            pan = input()
            print('camera_tilt: ')
            tilt = input()
            print('camera_zoom: ')
            zoom = input()
            print('record time (in second): ')
            rtime = input()
            print('Please choose point on map to save')
            call_server(choose,pos,pan,tilt,zoom,rtime,buffer)
        choose = 0

    elif choose == 4:
        rospy.loginfo('Delete Point')
        print('Point name to delete: ')
        pos = raw_input()
        call_server(choose,pos,0,0,0,0,0)
        choose = 0

    elif choose == 5:
        rospy.loginfo('Show Points')
        print('on or off: ')
        pos = raw_input()
        call_server(choose,pos,0,0,0,0,0)
        choose = 0
    
    elif choose == 6:
        rospy.loginfo("Back Home")
        call_server(choose,'',0,0,0,0,0)
        choose = 0

    else:
        choose = 0


if __name__ == '__main__':

    rospy.init_node('teach_point_client')
    rospy.loginfo('Beginnn Teach Point Client')
    while(1):
        main()
    
    rospy.spin()
