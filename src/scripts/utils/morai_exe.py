#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy,rospkg
from math import cos,sin,sqrt,pow,atan2,pi
import sys,os,time
import json
from lib.udp_parser import udp_parser,udp_sender

rospy.init_node('my_node', anonymous=True)

# config
param_filename = "params.txt"
if rospy.has_param('~morai_params'):
    param_filename = rospy.get_param('~morai_params')
network_filename = None
if rospy.has_param('~morai_network'):
    network_filename = rospy.get_param('~morai_network')
scenario_filename = None
if rospy.has_param('~morai_scenario'):
    scenario_filename = rospy.get_param('~morai_scenario')

#read params.txt

file = open(param_filename, 'r')
line=file.read()
params=line.split('\n')
for i in range(0,len(params)):
    params[i]=(params[i].split(':')[1].replace(" ","")).replace('\r','')


recive_user_ip = params[0]
recive_user_port = int(params[1])

request_dst_ip = params[2]
request_dst_port = int(params[3])


class sim_ctrl :

    def __init__(self):
        self.get_status = udp_parser(recive_user_ip, recive_user_port,'get_sim_status')
        self.set_status = udp_sender(request_dst_ip, request_dst_port,'set_sim_status')
        self.is_sim = False
        self.is_play = False
        self.is_print = False
        self.main_loop()

    def main_loop(self):
        while True:
            self.status_data=self.get_status.get_data()

            if not len(self.status_data)==0:
                self.data_platform = self.status_data[0]
                self.data_stage = self.status_data[1]
                self.data_status = self.status_data[2]

                self.command_platform = self.status_data[3]
                self.command_cmd = self.status_data[4]
                self.command_option = self.status_data[5]
                self.command_result = self.status_data[6]
                print("data", self.data_platform, self.data_stage, self.data_status, self.command_result, self.command_option)

                if self.data_platform == "0x01" and self.data_stage=="0x01" and not self.is_sim: #Launcher platform에서 로그인 전 상태
                    cmd_platform="0x01"
                    cmd_command="0x0001"
                    cmd_option=params[4]+"/"+params[5]#(ID/PW)
                    self.send_data([cmd_platform,cmd_command,cmd_option])#Login명령

                if self.data_platform=="0x01" and self.data_stage=="0x02" and not self.is_sim: #Launcher platform에서 로그인 후 상태
                    cmd_platform = "0x01"
                    cmd_command = "0x0002"
                    cmd_option = params[6]#simulator version
                    self.send_data([cmd_platform,cmd_command,cmd_option]) #version 선택 명령

                if self.data_platform=="0x01" and self.data_stage=="0x02" and self.data_status=="0x0001" and not self.is_sim: #Result = 성공
                    cmd_platform="0x01"
                    cmd_command="0x0004"
                    cmd_option=""
                    self.is_sim = True
                    self.send_data([cmd_platform,cmd_command,cmd_option]) #Simulator 실행 명령

                if self.data_platform=="0x02" and self.data_stage=="0x01" and self.data_status=="0x0001" and not self.is_print: #Simulator platform에서 로비 Stage의 대기상태
                    cmd_platform="0x02"
                    cmd_command="0x0001"
                    cmd_option=params[7]#(MAP/Vehicle)

                    self.send_data([cmd_platform,cmd_command,cmd_option]) #시뮬레이션/옵션 변경 실행 명령
                    print("choose map & vehicle")
                    self.is_print = True
                    break

                if self.data_platform=="0x02" and self.data_stage=="0x02" and (self.data_status=="0x0005" or self.data_status=="0x0001"):
                    break

                # if self.data_platform=="0x02" and self.data_stage=="0x02" and self.data_status=="0x0005" and not self.is_play: #Simulator platform에서 play상태
                #     if scenario_filename is None:
                #         break

                #     cmd_platform="0x02"
                #     cmd_command="0x0013"
                #     cmd_option=scenario_filename#(Scenario file name)

                #     cmd_option2="0x01 0x02 0x04 0x08 0x10 0x20" #option2 or 연산
                #     # 0x01 : Delete All
                #     # 0x02 : Ego Vehicle
                #     # 0x04 : Surround Vehicle
                #     # 0x08 : Pedestrian
                #     # 0x10 : Obstacle
                #     # 0x20: Pause
                #     temp_option=cmd_option2.split()
                #     for i in range(0,len(temp_option)):
                #         temp_option[i]=int(temp_option[i],0)
                #     result=temp_option[0]

                #     for i in range(1,len(temp_option)):
                #         result=(result|temp_option[i])
                #     cmd_option=cmd_option+"/"+str(result)

                #     time.sleep(1)
                #     self.send_data([cmd_platform,cmd_command,cmd_option]) #시뮬레이션/옵션 변경 실행 명령
                #     self.is_play = True

                # if self.data_platform=="0x02" and self.data_stage=="0x02" and self.data_status=="0x0001" and self.is_play: #Simulator platform에서 play상태
                #     if network_filename is None:
                #         break

                #     cmd_platform="0x02"
                #     cmd_command="0x0011"
                #     cmd_option=network_filename#(network file name)

                #     time.sleep(1)
                #     self.send_data([cmd_platform,cmd_command,cmd_option]) #시뮬레이션/옵션 변경 실행 명령

                #     break

                time.sleep(1)
            else :
                print("[NO Simulator Control Data]")
                time.sleep(0.5)

        print("end")

    def send_data(self, data):
        try:
            print("send",data)
            cmd_platform=int(data[0],0)
            cmd_command=int(data[1],0)
            cmd_option=data[2]
            self.set_status.send_data([cmd_platform,cmd_command,cmd_option])
        except ValueError:
            print("Invalid input")

if __name__ == "__main__":
    ctrl=sim_ctrl()

