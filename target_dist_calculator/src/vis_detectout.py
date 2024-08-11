#!/usr/bin/env python

import rospy
from target_dist_calculator.msg import DetectOut  # 请根据实际包名替换 custom_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CustomMessageSubscriber:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('custom_message_subscriber', anonymous=True)

        # 创建 CvBridge 对象
        self.bridge = CvBridge()

        # 订阅自定义消息话题
        self.subscriber = rospy.Subscriber('/target_dist_calculator/detect_out', DetectOut, self.callback)

    def callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 显示图像
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        CustomMessageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
