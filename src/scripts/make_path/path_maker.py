#!/usr/bin/env python3
import rospy
from morai_msgs.msg import EgoVehicleStatus
import json
import math

# 주행 경로를 저장할 리스트
path = []

# 마지막으로 기록된 위치를 저장할 변수
last_position = None

def calculate_distance(pos1, pos2):
    """두 위치 사이의 유클리드 거리를 계산합니다."""
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

def callback(data):
    global last_position
    # 현재 위치 정보
    position = data.position
    
    # 이전 위치가 없거나 현재 위치와의 차이가 0.5 이상인 경우에만 기록
    if last_position is None or calculate_distance(position, last_position) >= 0.5:
        # x, y, z 좌표를 리스트 형태로 저장
        path.append([position.x, position.y, position.z])
        last_position = position  # 마지막 위치 업데이트
        
        # 실시간으로 위치 정보를 출력 (선택적)
        rospy.loginfo("Recorded Position: x=%f, y=%f, z=%f", position.x, position.y, position.z)

def listener():
    # ROS 노드 초기화
    rospy.init_node('path_recorder', anonymous=True)

    # /Ego_topic 구독
    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, callback)

    # rospy.spin() 호출로 콜백 함수가 계속해서 호출될 수 있게 함
    rospy.spin()

    # 주행이 끝나고 노드가 종료될 때 경로를 파일로 저장
    with open('path.json', 'w') as f:
        json.dump(path, f)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass