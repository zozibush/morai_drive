#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, CtrlCmd
from autonomous_driving.vehicle_state import VehicleState
from autonomous_driving.perception.object_info import ObjectInfo
from autonomous_driving.config.config import Config


class RosManager:
    def __init__(self, autonomous_driving):
        self.autonomous_driving = autonomous_driving

        config = Config()
        rospy.init_node("morai_standard", anonymous=True)

        self.global_path = self.convert_to_ros_path(config["map"]["path"], '/map')

        self.sampling_rate = config["common"]["sampling_rate"]
        self.ros_rate = rospy.Rate(self.sampling_rate)
        self.count = 0

        self.vehicle_state = VehicleState()
        self.object_info_list = []

        self.is_status = False
        self.is_object_info = False

    def execute(self):
        print("start simulation")
        self._set_protocol()
        while not rospy.is_shutdown():
            if self.is_status and self.is_object_info:
                control_input, local_path, lattice_path = self.autonomous_driving.execute(
                    self.vehicle_state, self.object_info_list
                )
                self._send_data(control_input, local_path, lattice_path)
        print("end simulation")

    def _set_protocol(self):
        # publisher
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.lattice_path_pub =  [rospy.Publisher('/lattice_path_' + str(i), Path, queue_size = 1) for i in range(10)]

        # subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.vehicle_status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)

    def _send_data(self, control_input, local_path, lattice_path):
        self.ctrl_pub.publish(CtrlCmd(**control_input.__dict__))
        self.local_path_pub.publish(self.convert_to_ros_path(local_path, 'map'))
        self.odom_pub.publish(self.convert_to_odometry(self.vehicle_state))

        if self.count == self.sampling_rate:
            self.global_path_pub.publish(self.global_path)
            self.count = 0
        for i in range(min(10, len(lattice_path))):
            self.lattice_path_pub[i].publish(self.convert_to_ros_path(lattice_path[i], 'map'))

        self.count += 1
        self.ros_rate.sleep()

    @staticmethod
    def convert_to_ros_path(path, frame_id):
        ros_path = Path()
        ros_path.header.frame_id = frame_id
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(x=point.x, y=point.y, z=0)
            pose_stamped.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
            ros_path.poses.append(pose_stamped)

        return ros_path

    @staticmethod
    def convert_to_odometry(vehicle_state):
        odometry = Odometry()
        odometry.header.frame_id = 'map'
        odometry.child_frame_id = 'gps'

        quaternion = tf.transformations.quaternion_from_euler(0, 0, vehicle_state.yaw)

        odometry.pose.pose.position = Point(x=vehicle_state.position.x, y=vehicle_state.position.y, z=0)
        odometry.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        return odometry

    def vehicle_status_callback(self, data):
        self.vehicle_state = VehicleState(data.position.x, data.position.y, np.deg2rad(data.heading), data.velocity.x)
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (self.vehicle_state.position.x, self.vehicle_state.position.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.vehicle_state.yaw),
            rospy.Time.now(),
            "gps",
            "map"
        )
        self.is_status = True

    def object_info_callback(self, data):
        self.object_info_list = [
            ObjectInfo(data.position.x, data.position.y, data.velocity.x, data.type)
            for data in data.npc_list + data.obstacle_list + data.pedestrian_list
        ]
        self.is_object_info = True
