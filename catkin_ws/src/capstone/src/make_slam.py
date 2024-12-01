#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf import TransformListener
from sign_check import Sign
from lane_tracking import LaneTracking

class MarkerSlam:
    def __init__(self):
        rospy.init_node('marker_slam')
        
        # Aruco 마커 ID와 해당 위치를 저장하는 딕셔너리
        self.marker_positions = {}
        
        # ROS 카메라 구독
        self.image_sub = rospy.Subscriber('/stereo/left/image_raw', Image, self.image_callback)
        #표지판 ID 구독
        rospy.Subscriber('sign_id', Int32, self.sign_callback) 
        # 속도명령 퍼블리셔 구독
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # ROS와 OpenCV간 이미지 변환 브릿지
        self.bridge = CvBridge()
        # TF Listener : 현재 위치와 목표 위치 계산에 사용
        self.tf_listener = TransformListener()
        # 현재 감지된 표지판
        self.current_sign_id = 0

    def sign_callback(self, msg):
        """
        표지판 ID를 업데이트하는 콜백 함수
        """
        self.current_sign_id = msg.data  # 현재 감지된 표지판 ID 업데이트
        rospy.loginfo(f"현재 감지된 표지판 ID: {self.current_sign_id}")
        self.main_control.update_sign(self.current_sign_id)  # 제어 로직에 표지판 ID 전달

     def image_callback(self, msg):
            """
            카메라에서 이미지를 수신하여 처리하는 콜백 함수
            """
            try:
                # ROS 이미지 메시지를 OpenCV 이미지로 변환
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                rospy.logerr(f"이미지 변환 실패: {e}")
                return
    
            # 차선 추적 수행
            self.lane_tracker.adjust_hsv_based_on_brightness(frame)  # 조명에 따라 HSV 범위 조정
            lane_mask = self.lane_tracker.detect_yellow_lane(frame)  # 노란 차선 감지
            steering_angle = self.lane_tracker.compute_steering_angle(lane_mask)  # 스티어링 각도 계산
    
            # 계산된 스티어링 각도를 제어 로직에 전달
            self.main_control.update_steering(steering_angle)
    
            # 결과를 시각화하여 표시
            result = cv2.bitwise_and(frame, frame, mask=lane_mask)  # 차선 영역 강조
            cv2.putText(result, f"Steering Angle: {steering_angle:.2f}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)  # 스티어링 각도 표시
            cv2.putText(result, f"Sign ID: {self.current_sign_id}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)  # 표지판 ID 표시
    
            cv2.imshow("Lane and Sign Detection", result)  # 결과를 OpenCV 창에 출력
            cv2.waitKey(1)
    
        def navigate_to_marker(self, marker_id):
            """
            특정 Aruco 마커 ID 위치로 로봇을 이동
            """
            if marker_id not in self.marker_positions:
                rospy.logwarn(f"Marker {marker_id} 위치를 알 수 없습니다.")
                return
    
            # 목표 위치를 딕셔너리에서 가져옴
            target_pos = self.marker_positions[marker_id]
            rospy.loginfo(f"목표 위치로 이동: {target_pos}")
    
            rate = rospy.Rate(10)  # 루프 실행 주기 (10Hz)
            while not rospy.is_shutdown():
                try:
                    # TF를 사용하여 현재 로봇 위치를 '/map' 기준으로 가져옴
                    (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    x, y = trans[0], trans[1]
    
                    # 현재 위치와 목표 위치 간의 차이 계산
                    error_x = target_pos[0] - x
                    error_y = target_pos[1] - y
    
                    # 목표 위치에 충분히 가까워지면 멈춤
                    if abs(error_x) < 0.1 and abs(error_y) < 0.1:
                        rospy.loginfo("목표 위치에 도달.")
                        self.cmd_vel_pub.publish(Twist())  # 정지 명령
                        break
    
                    # 속도 명령 생성
                    twist = Twist()
                    twist.linear.x = 0.2 * np.sqrt(error_x**2 + error_y**2)  # 거리 비례 선형 속도
                    twist.angular.z = 1.0 * np.arctan2(error_y, error_x)    # 회전 비례 각속도
                    self.cmd_vel_pub.publish(twist)
    
                except Exception as e:
                    rospy.logwarn(f"TF Transform 실패: {e}")
    
                rate.sleep()


if __name__ == '__main__':
    try:
        # MarkerSLAM 클래스 실행
        node = MarkerSLAM()
        rospy.loginfo("Marker SLAM 노드 실행 중...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    
