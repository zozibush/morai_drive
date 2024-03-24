#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,pi,sqrt,pow,atan2
import numpy as np
from ..localization.point import Point

class LatticePlanner:
    def __init__(self):
        pass

    def get_lattice_path(self, local_path, status_msg, object_data):

        if self.checkObject(local_path, object_data):
            print("obstacle detecting.."+str(self.object_type))
            lattice_path = self.latticePlanner(local_path, status_msg)
            lattice_path_index = self.collision_check(object_data, lattice_path)

            return lattice_path[lattice_path_index], lattice_path
        else:
            return local_path, []

    def checkObject(self, ref_path, object_data):
        is_crash = False
        for obstacle in object_data:
            object_info = obstacle['object_info']
            object_type = object_info.type
            distance_threshold = 5

            for point in ref_path:
                distance_from_path = point.distance(object_info.position)
                if distance_from_path < distance_threshold:
                    is_crash = True
                    self.object_type = object_type
                    break

        return is_crash

    def collision_check(self, object_data, out_path):
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택

        selected_lane = -1
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path

        for obstacle in object_data:
            object_info = obstacle['object_info']
            for path_num in range(len(out_path)):
                for point in out_path[path_num]:
                    dis = point.distance(object_info.position)
                    if dis < 4.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))

        return selected_lane

    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity * 3.6

        look_distance = int(vehicle_velocity * 0.2 * 2)

        if look_distance < 20 :
            look_distance = 20

        if len(ref_path) > look_distance :
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            """

            global_ref_start_point      = (ref_path[0].x, ref_path[0].y)
            global_ref_start_next_point = (ref_path[1].x, ref_path[1].y)

            global_ref_end_point = (ref_path[look_distance * 2].x, ref_path[look_distance * 2].y)

            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []

            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])

            #TODO: (4) Lattice 충돌 회피 경로 생성
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
            '''

            for end_point in local_lattice_points :
                lattice_path = []
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs,int(x_num)) :
                    x.append(i*x_interval)

                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)

                # 3차 곡선 계획
                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = Point(global_result[0][0], global_result[1][0])
                    # read_pose.x= global_result[0][0]
                    # read_pose = global_result[1][0]
                    # lattice_path.poses.append(read_pose)
                    lattice_path.append(read_pose)

                out_path.append(lattice_path)

            #Add_point
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path) )

            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path):
                    tmp_theta = atan2(ref_path[i + 1].y - ref_path[i].y,ref_path[i + 1].x - ref_path[i].x)                    
                    tmp_translation = [ref_path[i].x,ref_path[i].y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = Point(global_result[0][0], global_result[1][0])
                        out_path[lane_num].append(read_pose)

        return out_path
