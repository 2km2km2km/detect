#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class OdometryHandler {
public:
    OdometryHandler(ros::NodeHandle& nh) {
        // 订阅 odometry 消息
        odom_sub_ = nh.subscribe("/Odometry", 10, &OdometryHandler::odomCallback, this);
    }

private:
    ros::Subscriber odom_sub_;
    tf2::Transform transform_;  // 用于维护两个 frame 之间的关系
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::cout<<"get odom"<<std::endl;
        // 提取位置信息
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        // 提取四元数（姿态）信息
        tf2::Quaternion q;
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        q.setW(msg->pose.pose.orientation.w);

        // 更新两个 frame 之间的变换关系
        transform_.setOrigin(tf2::Vector3(x, y, z));
        transform_.setRotation(q);

        // 广播变换
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "body";
        transform_stamped.child_frame_id = "world";  // 假设子 frame 是 base_link
        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = z;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // 发送变换
        tf_broadcaster_.sendTransform(transform_stamped);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_handler");
    ros::NodeHandle nh;

    OdometryHandler odometry_handler(nh);

    ros::spin();

    return 0;
}
