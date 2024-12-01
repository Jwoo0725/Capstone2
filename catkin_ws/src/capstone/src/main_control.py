#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
from lane_tracking import LaneTracking


class Controller():

    def __init__(self): 
        rospy.init_node('main_control_node')

        rospy.Subscriber('/stereo/left/image_raw', Image, self.image_callback)

        #rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)

        self.lane_tracker = LaneTracking()

        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # 조명 변화에 따른 HSV 범위 조정
        self.lane_tracker.adjust_hsv_based_on_brightness(frame)

        # 노란색 차선 감지
        mask = self.lane_tracker.detect_yellow_lane(frame)

        # 스티어링 각도 계산
        steering_angle = self.lane_tracker.compute_steering_angle(mask)

        # 차량 제어
        #self.drive(steering_angle)

        # 결과 시각화
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.putText(result, f"Steering Angle: {steering_angle:.2f} deg", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Yellow Lane Detection", result)
        cv2.waitKey(1)

    def child_sign_callback(self, _data):
        try :
            # : {}".format(_data.data))

            if _data.data == 3:
                self.child_cnt += 1
                if self.child_cnt >=20 :
                    self.sign_data = _data.data
                    self.slow_down_flag = 1
                    self.child_cnt = 0
            else :
                self.sign_data = 0
                # self.slow_down_flag = 0
            #rospy.loginfo(" sign data_callback  : {}".format(self.sign_data))

            #if _data.data == 3:
            #    self.slow_down_flag = 1
               
        except :
            pass

if __name__ == '__main__':
    try:
        node = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass