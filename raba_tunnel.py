#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

class WallFollow:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('wall_follow', anonymous=True)
        
        # 라이다 구독자 및 조향 발행자 초기화
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.steering_publisher = rospy.Publisher('tunnel_steering', Float64, queue_size=10)
        
        # 조향 제어 상수 및 초기값 설정
        self.steering_gain = 1      # 비례제어 상수 (상황에 맞게 튜닝 필요)
        self.initial_steering = 0.5  # 초기 조향값
        self.min_steering = 0.1      # 최소 조향값
        self.max_steering = 0.9      # 최대 조향값
        self.detection_range = 0.50   # 감지 범위 30cm로 제한

        # PID 제어 변수
        self.kp = 3.0     # 비례 제어 상수
        self.ki = 0.0     # 적분 제어 상수
        self.kd = 0.03     # 미분 제어 상수
        self.previous_error = 0.0
        self.integral = 0.0

    def pid_control(self, error):
        # PID 계산
        self.integral += error  # 적분
        derivative = error - self.previous_error  # 미분
        self.previous_error = error  # 이전 에러 업데이트
        
        # PID 출력
        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return control_output

    def lidar_callback(self, msg):
        # 각도 단위 계산
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        degrees = [math.degrees(angle) for angle in angles]  # 라디안을 각도로 변환
        
        # 좌측 벽 (90도~135도)
        left_wall_indices = [i for i, degree in enumerate(degrees) if -180 <= degree <= -140]
        left_wall_ranges = [msg.ranges[i] for i in left_wall_indices if msg.ranges[i] < self.detection_range]
        left_wall_distance = min(left_wall_ranges) if left_wall_ranges else self.detection_range
        
        # 우측 벽 (0도~45도)
        right_wall_indices = [i for i, degree in enumerate(degrees) if 140 <= degree <= 180]
        right_wall_ranges = [msg.ranges[i] for i in right_wall_indices if msg.ranges[i] < self.detection_range]
        right_wall_distance = min(right_wall_ranges) if right_wall_ranges else self.detection_range
        
        # 거리 차이 계산
        distance_diff = right_wall_distance - left_wall_distance
        
        # PID 제어를 사용하여 조향 각도 계산
        pid_output = self.pid_control(distance_diff)
        steering_angle = self.initial_steering + pid_output
        
        # 조향 값 제한
        steering_angle = max(self.min_steering, min(self.max_steering, steering_angle))
        
        # 조향 값 발행
        steering_msg = Float64()
        steering_msg.data = steering_angle
        self.steering_publisher.publish(steering_msg)
        
        # 디버그 출력
        rospy.loginfo(f'Left Wall Distance: {left_wall_distance:.2f}, Right Wall Distance: {right_wall_distance:.2f}, Distance Diff: {distance_diff:.2f}, Steering Angle: {steering_angle:.2f}')

def main():
    wall_follow = WallFollow()
    rospy.spin()

if __name__ == '__main__':
    main()
