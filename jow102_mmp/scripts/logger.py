# -*- coding: utf-8 -*-

#! /usr/bin/env python
import rospy
import geometry_msgs
from gazebo_msgs.msg import ModelStates

import csv

robot_name = 'turtlebot3_waffle_pi'
fieldnames = ['timestamp', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']


def cb_gazebo_states(msg):
    global writer
    #extract car pose from msg
    if robot_name in msg.name:
        index = msg.name.index(robot_name)
        pose = msg.pose[index]  #position and orientation of car
        twist = msg.twist[index]
        
        data = [rospy.get_time(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        writer.writerow(data)

        rospy.logdebug("position=(%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z)
        rospy.logdebug("orientation=(%.2f, %.2f, %.2f, %.2f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    else:
        rospy.logwarn("Robot model not found, %s", len(msg.model_name))
        return



if __name__ == '__main__':
    global writer
    rospy.init_node('robot_logger')
    
    
    # open the file in the write mode
    csv_file = open('./robot_log.csv', 'w')
    # create the csv writer
    writer = csv.writer(csv_file)
    writer.writerow(fieldnames)
    
    
    rospy.Subscriber('/gazebo/model_states',
                     ModelStates,
                     cb_gazebo_states)

    rospy.spin()
