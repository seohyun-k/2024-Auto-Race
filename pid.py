import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64, Int64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 비례 상수
        self.ki = ki  # 적분 상수
        self.kd = kd  # 미분 상수
        self.previous_error = 0  # 이전 오차
        self.integral = 0  # 적분 값

    def calculate(self, error):
        # 비례 제어
        proportional = self.kp * error
        
        # 적분 제어
        self.integral += error
        integral = self.ki * self.integral
        
        # 미분 제어
        derivative = self.kd * (error - self.previous_error)
        
        # 이전 오차 업데이트
        self.previous_error = error
        
        # PID 제어 값 계산
        output = proportional + integral + derivative
        return output

class lane_detect:
    def __init__(self):
        rospy.init_node('lane_detection')

        self.steer_angle_pub = rospy.Publisher('steering_angle', Float64, queue_size=10)
        self.status_pub = rospy.Publisher('status', Int64, queue_size=10)
        self.child_pub = rospy.Publisher('child', Int64, queue_size=10)
        self.stop_pub = rospy.Publisher('stop', Bool, queue_size=10)
        self.lane_pub = rospy.Publisher('lane', Image, queue_size=10)
        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.image_CB)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(100)

        ## 기본 변수 설정
        self.steer_msg = Float64()
        self.status_msg = Int64()
        self.stop_msg = Bool()
        self.lane_msg = Image()
        
        ## Bird-eye View를 위한 포인트 설정
        self.left_top = (150, 320)
        self.left_bottom = (80, 480)
        self.right_top = (550, 320)
        self.right_bottom = (640, 480)
        self.src_points = np.float32([self.left_top, self.left_bottom, self.right_bottom, self.right_top])
        self.dst_points = np.float32([(160, 0), (160, 480), (480, 480), (480, 0)])
        
        ## PID 제어기 초기화 (kp, ki, kd 값 설정)
        self.pid_controller = PIDController(kp=0.06, ki=0.00001, kd=0.0000)

        ## 기본 각도 및 조향값 설정
        self.aver_ang = 90
        self.aver_steer = 0.5
        self.ob_flag = 0
        self.stop_count=0
        self.stop_flag= False

    def image_CB(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            bird_view = self.bird_eye_view(frame, height, width)
            cv2.imshow("roi", bird_view)
            self.find_lanes(bird_view)

            self.status_pub.publish(self.status_msg)
            self.child_pub.publish(self.child_msg)  # child 메시지 발행
            key = cv2.waitKey(1)
            if key == ord('q'):
                rospy.signal_shutdown("User requested shutdown")
        except Exception as e:
            print("Error:", e)

    def bird_eye_view(self, frame, height, width):
        M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        warp_frame = cv2.warpPerspective(frame, M, (width, height))
        return warp_frame

    def detect_child_zone(self, hsv):
        # 어린이 보호구역 빨간색 도로 범위 (HSV 색상 범위)
        lower_red = np.array([0, 100, 60])
        upper_red = np.array([10, 255, 255])  # 빨간색 범위
        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        # 빨간색 영역의 크기 계산
        red_area = cv2.countNonZero(red_mask)

        # 일정 면적 이상일 때만 어린이 보호구역으로 설정 (예: 1000개 픽셀 이상)
        area_threshold = 1000  # 예시: 1000픽셀 이상
        child_msg = Int64()

        if red_area > area_threshold:  # 빨간색 도로 영역이 기준 면적 이상이면 어린이 보호구역
            child_msg.data = 1
        else:
            child_msg.data = 0

        return child_msg

    def find_lanes(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 노란색 차선 범위 (HSV 색상 범위)
        lower_yellow = np.array([20, 50, 60])
        upper_yellow = np.array([80, 255, 255])
        
        # 노란색 차선 마스크 생성
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 어린이 보호구역 감지
        self.child_msg = self.detect_child_zone(hsv)

        # 노란색 차선 마스크를 이용한 차선 추출
        result = cv2.bitwise_and(image, image, mask=yellow_mask)

        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, threshold1=50, threshold2=150)
        
        height, width = edges.shape[:2]
        
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=100, maxLineGap=150)  # Hough 파라미터 조정
        line_image = np.zeros_like(image)

        stop_detected_time = None
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)

                if x2 - x1 != 0:
                    slope = (x2 - x1) / (y2 - y1)
                    stop_slope=(y2-y1)/(x2-x1)
                    slope_degree = np.arctan(slope) * 180 / np.pi   

                    if abs(stop_slope) < 0.1 and y1 > 240 and y2 > 240 and self.stop_count == 0:
                        if not self.stop_msg.data:
                            self.stop_msg.data = True
                            stop_detected_time = time.time()
                            print("stop")
                            self.stop_count = 1
                    
                    self.stop_pub.publish(self.stop_msg)

                    # PID 제어기를 이용하여 조향값 계산
                    error = slope_degree
                    print(error)
                    pid_output = self.pid_controller.calculate(error)
                    self.steer_msg.data = 0.5 - pid_output  # PID 제어 값 반영

                    self.steer_msg.data = min(max(self.steer_msg.data, 0), 1)
    
        # 만약 라인을 검출하지 못했다면 steer_ang 값을 0.5로 설정 (기본값)
        if lines is None:
            self.steer_msg.data = 0.5

        self.steer_angle_pub.publish(self.steer_msg)
        self.status_pub.publish(self.status_msg)

        result = cv2.addWeighted(image, 0.8, line_image, 1, 0)
        self.lane_msg = self.bridge.cv2_to_imgmsg(result, encoding="bgr8")
        self.lane_pub.publish(self.lane_msg)
        "cv2.imshow(result, result)"
        self.rate.sleep()

def main():
    start = lane_detect()
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
