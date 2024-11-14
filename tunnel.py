#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class WallFollow:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('wall_follow', anonymous=True)
        
        # 라이다 구독자 및 조향 발행자 초기화
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.steering_publisher = rospy.Publisher('tunnel_steering', Float64, queue_size=10)
        
        # 조향 제어 상수 및 초기값 설정
        self.steering_gain = 0.8     # 비례제어 상수 (상황에 맞게 튜닝 필요)
        self.initial_steering = 0.55  # 초기 조향값
        self.min_steering = 0.25      # 최소 조향값
        self.max_steering = 0.85      # 최대 조향값
        self.detection_range = 0.5   # 감지 범위 50cm로 제한

    def lidar_callback(self, msg):
        # 좌측 벽 (90도~135도) 범위에서 50cm 이내 거리만 고려
        left_wall_ranges = [r for r in msg.ranges[135:180] if r < self.detection_range]
        left_wall_distance = min(left_wall_ranges) if left_wall_ranges else self.detection_range
        
        # 우측 벽 (0도~45도) 범위에서 50cm 이내 거리만 고려
        right_wall_ranges = [r for r in msg.ranges[1035:1100] if r < self.detection_range]
        right_wall_distance = min(right_wall_ranges) if right_wall_ranges else self.detection_range
        
        # 거리 차이 계산
        distance_diff = right_wall_distance - left_wall_distance
        
        # 조향 각도 계산 (거리가 먼 쪽으로 이동하도록 설정)
        steering_angle = self.initial_steering + (distance_diff * self.steering_gain)
        
        # 조향 값 제한
        steering_angle = max(self.min_steering, min(self.max_steering, steering_angle))
        
        # 조향 값 발행
        steering_msg = Float64()
        steering_msg.data = steering_angle
        self.steering_publisher.publish(steering_msg)
        
        # 디버그 출력
        rospy.loginfo(f'Left Wall Distance: {left_wall_distance:.2f}, Right Wall Distance: {right_wall_distance:.2f}, Steering Angle: {steering_angle:.2f}')

def main():
    wall_follow = WallFollow()
    rospy.spin()

if __name__ == '__main__':
    main()

