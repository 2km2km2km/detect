import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

class DepthToPointCloud:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('depth_to_point_cloud_node', anonymous=True)
        
        # 创建 CvBridge 对象
        self.bridge = CvBridge()
        
        # 订阅深度图和相机内参话题
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        # 发布点云话题
        self.point_cloud_pub = rospy.Publisher('/cloud_registered_body', PointCloud2, queue_size=10)
        
        # 相机内参矩阵
        self.intrinsic_matrix = None

    def camera_info_callback(self, msg):
        # 从 CameraInfo 消息中提取相机内参矩阵
        self.intrinsic_matrix = np.array(msg.K).reshape(3, 3)
    
    def depth_callback(self, msg):
        if self.intrinsic_matrix is None:
            rospy.logwarn("等待相机内参矩阵...")
            return
        
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        print("get depth")
        # 将深度图转换为点云
        point_cloud = self.depth_to_point_cloud(depth_image, self.intrinsic_matrix)
        
        # 发布点云
        self.point_cloud_pub.publish(point_cloud)

    def depth_to_point_cloud(self, depth_image, intrinsic_matrix):
        height, width = depth_image.shape
        fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
        cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]

        # 生成网格索引
        x = np.linspace(0, width - 1, width)
        y = np.linspace(0, height - 1, height)
        x, y = np.meshgrid(x, y)

        # 将深度图展开为一维数组
        z = depth_image.flatten()

        # 将像素坐标转换为相机坐标系下的坐标
        x = (x.flatten() - cx) * z / fx
        y = (y.flatten() - cy) * z / fy

        # 组合为点云
        points = np.vstack((x, y, z)).T

        # 将点云转换为 ROS 消息
        point_cloud_msg = self.convert_to_point_cloud2(points)

        return point_cloud_msg

    def convert_to_point_cloud2(self, points):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_link'  # 请根据您的实际 frame_id 进行设置
        cloud_data = pc2.create_cloud_xyz32(header, points)
        return cloud_data

if __name__ == "__main__":
    try:
        DepthToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
