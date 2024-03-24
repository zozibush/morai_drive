#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def load_path_from_json(file_path):
    with open(file_path, 'r') as f:
        waypoints = json.load(f)
    return waypoints

def convert_to_ros_path(waypoints, frame_id='/map'):
    path_msg = Path()
    path_msg.header.frame_id = frame_id

    for wp in waypoints:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position.x = wp[0]
        pose_stamped.pose.position.y = wp[1]
        pose_stamped.pose.position.z = wp[2]
        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = 1
        path_msg.poses.append(pose_stamped)

    return path_msg

def publish_path():
    rospy.init_node('path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    file_path = 'path.json'  # 경로 파일 위치
    waypoints = load_path_from_json(file_path)
    ros_path = convert_to_ros_path(waypoints)

    while not rospy.is_shutdown():
        path_pub.publish(ros_path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass