#!/usr/bin/env python
# -*- coding: utf-8 -*-
from .perception.forward_object_detector import ForwardObjectDetector
from .localization.path_manager import PathManager
from .planning.adaptive_cruise_control import AdaptiveCruiseControl
from .planning.lattice_planner import LatticePlanner
from .control.pure_pursuit import PurePursuit
from .control.pid import Pid
from .control.control_input import ControlInput
from .config.config import Config

from .mgeo.calc_mgeo_path import mgeo_dijkstra_path

import sys

class AutonomousDriving:
    def __init__(self):
        config = Config()

        if config["map"]["use_mgeo_path"]:
            mgeo_path = mgeo_dijkstra_path(config["map"]["name"])
            self.path = mgeo_path.calc_dijkstra_path_waypoints(config["map"]["mgeo"]["start_node"], config["map"]["mgeo"]["end_node"])
            self.path_manager = PathManager(
                self.path, config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )

        else:
            self.path = config["map"]["path"]
            self.path_manager = PathManager(
                self.path , config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )

        self.path_manager.set_velocity_profile(**config['planning']['velocity_profile'])
        if config["map"]["use_mgeo_path"]:
            print("use mgeo path")
        print(len(self.path))

        self.forward_object_detector = ForwardObjectDetector()

        self.adaptive_cruise_control = AdaptiveCruiseControl(
            vehicle_length=config['common']['vehicle_length'], **config['planning']['adaptive_cruise_control']
        )
        self.pid = Pid(sampling_time=1/float(config['common']['sampling_rate']), **config['control']['pid'])
        self.pure_pursuit = PurePursuit(
            wheelbase=config['common']['wheelbase'], **config['control']['pure_pursuit']
        )
        self.lattice_planner = LatticePlanner()

    def execute(self, vehicle_state, dynamic_object_list):
        # 현재 위치 기반으로 local path과 planned velocity 추출
        local_path, planned_velocity = self.path_manager.get_local_path(vehicle_state)

        HS = 0.03
        dis = local_path[0].distance(local_path[-1]) - planned_velocity * len(local_path) * HS
        # print(round(local_path[0].distance(local_path[-1]),0), round(planned_velocity * len(local_path) * HS, 0))
        if dis > 0:
            planned_velocity = min(30, planned_velocity)

        # 전방 장애물 인지
        self.forward_object_detector._dynamic_object_list = dynamic_object_list
        object_info_dic_list = self.forward_object_detector.detect_object(vehicle_state)

        # lattice palnner
        lattice_path = []
        local_path, lattice_path = self.lattice_planner.get_lattice_path(local_path, vehicle_state, object_info_dic_list)

        # adaptive cruise control를 활용한 속도 계획
        self.adaptive_cruise_control.check_object(local_path, object_info_dic_list)
        target_velocity = self.adaptive_cruise_control.get_target_velocity(vehicle_state.velocity, planned_velocity)

        # 속도 제어를 위한 PID control
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)
        # target velocity가 0이고, 일정 속도 이하일 경우 full brake를 하여 차량을 멈추도록 함.
        if round(target_velocity) == 0 and vehicle_state.velocity < 2:
            acc_cmd = -1.

        if round(vehicle_state.position[0]) > 138 and (round(vehicle_state.position[1]) < -102 and round(vehicle_state.position[1]) > -106):
            acc_cmd = -1.
            print("approach end point. Brake")

        # 경로 추종을 위한 pure pursuit control
        self.pure_pursuit.path = local_path
        self.pure_pursuit.vehicle_state = vehicle_state
        steering_cmd = self.pure_pursuit.calculate_steering_angle()

        return ControlInput(acc_cmd, steering_cmd), local_path, lattice_path
