    #include <ros/ros.h>
    #include <sensor_msgs/Image.h>
    #include <nav_msgs/Odometry.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <custom_msgs/BoundingBoxArray.h>
    #include <custom_msgs/TargetCoordinate.h>
    #include <geometry_msgs/Point.h>
    #include <cv_bridge/cv_bridge.h>
    #include <image_transport/image_transport.h>
    #include <sensor_msgs/image_encodings.h>
    #include <opencv2/opencv.hpp>
    #include <tf/transform_listener.h>
    #include <std_msgs/Header.h>
    #include <geometry_msgs/PointStamped.h>
    #include <tf/transform_broadcaster.h>
    #include <pcl/point_cloud.h>
    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/point_types.h>
    #include <target_dist_calculator/DetectOut.h>

    #include <Eigen/Dense>
    #include <fstream>
    #include <cmath>
    #include <deque>
    #include <chrono>

    #include <iostream>
    #include <vector>
    #include <pcl/io/pcd_io.h>
    #include <pcl/common/common.h>
    #include <yaml-cpp/yaml.h>
    
    using namespace std;
    using namespace cv;

    class TargetDistanceCalculator {
    public:
        TargetDistanceCalculator(ros::NodeHandle nh) {
            nh_ = nh;
            // 获取ROS参数
            nh_.param("camera_type", camera_type_, 1);  // 1: 大无人机，0: 小无人机
            nh_.param("raw_img", publish_raw_, true);
            nh_.param("dist_thre_add_point", dist_thre_add_point_, 0.1);
            nh_.param("dist_thre_final", dist_thre_final_, 0.1);
            nh_.param("dist_use_lidar", dist_use_lidar_, 5.0);
            nh_.param("person_width", person_width_, 0.0);
            nh_.param("person_heidht", person_heidht_, 0.0);
            nh_.param("width_rate", width_rate_, 0.0);
            nh_.param("height_rate", height_rate_, 0.0);
            nh_.param("config_path", config_path,std::string("config_path"));
            nh_.param("czd_save_path", czd_save_path,std::string("czd_save_path"));
            nh_.param("czd_img_save_path", czd_img_save_path,std::string("czd_img_save_path"));
            nh_.param("pc_topic", pc_topic,std::string("pc_topic"));
            // 初始化相机矩阵和畸变系数
            camera_matrix_ = (Mat_<double>(3, 3) << 376.824, 0.0, 316.4527,
                                                0.0, 377.400, 242.4085,
                                                0.0, 0.0, 1.0);
            dist_coeffs_ = (Mat_<double>(1, 5) << -0.00917852, -0.00467241, -0.00038472,  0.00083527, 0.000000);

            // 读取参数
            loadYamlFile(config_path);

            // 初始化图像、目标坐标、点云数据订阅
            image_subs_["CAM_A"] = nh_.subscribe<sensor_msgs::Image>(img_topic_["CAM_A"], 1, boost::bind(&TargetDistanceCalculator::imageCallback, this, _1, "CAM_A"));
            image_subs_["CAM_B"] = nh_.subscribe<sensor_msgs::Image>(img_topic_["CAM_B"], 1, boost::bind(&TargetDistanceCalculator::imageCallback, this, _1, "CAM_B"));
            image_subs_["CAM_C"] = nh_.subscribe<sensor_msgs::Image>(img_topic_["CAM_C"], 1, boost::bind(&TargetDistanceCalculator::imageCallback, this, _1, "CAM_C"));
            image_subs_["CAM_D"] = nh_.subscribe<sensor_msgs::Image>(img_topic_["CAM_D"], 1, boost::bind(&TargetDistanceCalculator::imageCallback, this, _1, "CAM_D"));

            bbox_subs_["CAM_A"] = nh_.subscribe<custom_msgs::BoundingBoxArray>(bbox_topic_["CAM_A"], 1, boost::bind(&TargetDistanceCalculator::bboxCallback, this, _1, "CAM_A"));
            bbox_subs_["CAM_B"] = nh_.subscribe<custom_msgs::BoundingBoxArray>(bbox_topic_["CAM_B"], 1, boost::bind(&TargetDistanceCalculator::bboxCallback, this, _1, "CAM_B"));
            bbox_subs_["CAM_C"] = nh_.subscribe<custom_msgs::BoundingBoxArray>(bbox_topic_["CAM_C"], 1, boost::bind(&TargetDistanceCalculator::bboxCallback, this, _1, "CAM_C"));
            bbox_subs_["CAM_D"] = nh_.subscribe<custom_msgs::BoundingBoxArray>(bbox_topic_["CAM_D"], 1, boost::bind(&TargetDistanceCalculator::bboxCallback, this, _1, "CAM_D"));

            lidar_sub_ = nh_.subscribe(pc_topic, 1, &TargetDistanceCalculator::lidarCallback, this);
            
            odometry_sub_ = nh_.subscribe("/Odometry", 1, &TargetDistanceCalculator::odometryCallback, this);

            // 初始化发布器
            target_pubs_coord_["CAM_A"] = nh_.advertise<custom_msgs::TargetCoordinate>("/detection/usb_cam_A/target_coordinates", 10);
            target_pubs_coord_["CAM_B"] = nh_.advertise<custom_msgs::TargetCoordinate>("/detection/usb_cam_B/target_coordinates", 10);
            target_pubs_coord_["CAM_C"] = nh_.advertise<custom_msgs::TargetCoordinate>("/detection/usb_cam_C/target_coordinates", 10);
            target_pubs_coord_["CAM_D"] = nh_.advertise<custom_msgs::TargetCoordinate>("/detection/usb_cam_D/target_coordinates", 10);

            target_pubs_img_with_coord_["CAM_A"] = nh_.advertise<sensor_msgs::Image>("/detection/usb_cam_A/target_img_with_coord", 10);
            target_pubs_img_with_coord_["CAM_B"] = nh_.advertise<sensor_msgs::Image>("/detection/usb_cam_B/target_img_with_coord", 10);
            target_pubs_img_with_coord_["CAM_C"] = nh_.advertise<sensor_msgs::Image>("/detection/usb_cam_C/target_img_with_coord", 10);
            target_pubs_img_with_coord_["CAM_D"] = nh_.advertise<sensor_msgs::Image>("/detection/usb_cam_D/target_img_with_coord", 10);

            target_pubs_coord_world_.push_back(nh_.advertise<sensor_msgs::PointCloud2>("/detection/target_coordinates/people", 1000));
            target_pubs_coord_world_.push_back(nh_.advertise<sensor_msgs::PointCloud2>("/detection/target_coordinates/drone", 1000));
            target_pubs_coord_world_.push_back(nh_.advertise<sensor_msgs::PointCloud2>("/detection/target_coordinates/box", 1000));

            target_pub_bearing_camera_ = nh_.advertise<geometry_msgs::Point>("/detection/target_bearing", 10);

            cam_line_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("line_segment_point_cloud", 10);
            closest_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("closest_point", 10);
            closest_point_wh_pub_ = nh_.advertise<geometry_msgs::PointStamped>("closest_point_wh", 10);
            pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 10);
            out_info_pub_ = nh_.advertise<target_dist_calculator::DetectOut>("detect_out", 10);
        }

    private:
        ros::NodeHandle nh_;
        map<string, ros::Subscriber> image_subs_;
        map<string, ros::Subscriber> bbox_subs_;
        ros::Subscriber lidar_sub_, odometry_sub_;

        map<string, ros::Publisher> target_pubs_coord_;
        map<string, ros::Publisher> target_pubs_img_with_coord_;
        vector<ros::Publisher> target_pubs_coord_world_;
        ros::Publisher target_pub_bearing_camera_;
        ros::Publisher cam_line_pub_, closest_point_pub_, closest_point_wh_pub_, pc_pub_, out_info_pub_;

        int camera_type_;
        bool publish_raw_;
        double dist_thre_add_point_, dist_thre_final_, dist_use_lidar_;
        double person_width_, person_heidht_, width_rate_, height_rate_;
        Mat camera_matrix_, dist_coeffs_;
        // map<string, Mat> rotation_matrix_, translation_vector_;
        map<string, Eigen::Matrix3d> rotation_matrix_;
        map<string, Eigen::Vector3d> translation_vector_;
        map<string, string> img_topic_;
        map<string, string> bbox_topic_;

        bool received_bbox_ = false;
        bool received_img_ = false;
        bool got_lidar_ = false;
        deque<sensor_msgs::PointCloud2> point_cloud_cache_;
        deque<nav_msgs::Odometry> odometry_cache_;

        map<string, Mat> images_;
        map<string, custom_msgs::BoundingBoxArray> bboxes_;
        
        string config_path, czd_save_path, czd_img_save_path, pc_topic;
        
        template <typename MsgType>
        int find_closest_diff(const std::deque<MsgType>& data_list, const ros::Time& target_time) {
            int closest_idx = -1;
            ros::Time closest_time;
            double min_diff = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < data_list.size(); ++i) {
                double time_diff = std::abs(data_list[i].header.stamp.toSec() - target_time.toSec());
                if (time_diff < min_diff) {
                    min_diff = time_diff;
                    closest_idx = static_cast<int>(i);
                    closest_time = data_list[i].header.stamp;
                }
            }
            return closest_idx;
        }

        void loadYamlFile(const std::string& file_path) {
            YAML::Node config = YAML::LoadFile(file_path);

            for (const auto& item : config) {
                std::string key = item.first.as<std::string>();

                YAML::Node rotation_node = item.second["rotation"];
                Eigen::Matrix3d rotation_matrix;
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        rotation_matrix(i, j) = rotation_node[i][j].as<double>();
                    }
                }
                rotation_matrix_[key] = rotation_matrix;

                YAML::Node translation_node = item.second["translation"];
                Eigen::Vector3d translation_vector;
                for (int i = 0; i < 3; ++i) {
                    translation_vector(i) = translation_node[i].as<double>();
                }
                translation_vector_[key] = translation_vector;
                
                // 读取img_topic
                YAML::Node img_topic_node = item.second["img_topic"];
                if (img_topic_node) {
                    img_topic_[key] = img_topic_node.as<std::string>();
                }

                // 读取bbox_topic
                YAML::Node bbox_topic_node = item.second["bbox_topic"];
                if (bbox_topic_node) {
                    bbox_topic_[key] = bbox_topic_node.as<std::string>();
                }
            }
        }

        void publish_cam_line(const Eigen::Vector3d& start_point, const Eigen::Vector3d& direction_vector, const ros::Time& stamp) {
            int num_points = 100;
            double length = 15.0;
            Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(num_points, 0, length);
            pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            for (int i = 0; i < num_points; ++i) {
                Eigen::Vector3d p = start_point + t(i) * direction_vector;
                pcl::PointXYZ point;
                point.x = p.x();
                point.y = p.y();
                point.z = p.z();
                line_cloud->push_back(point);
            }
            // 创建点云数据
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*line_cloud, cloud_msg);
            cloud_msg.header.stamp = stamp;
            cloud_msg.header.frame_id = "body";
            cam_line_pub_.publish(cloud_msg);
        }


        std::vector<pcl::PointXYZ> samplePoints(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, float length, int num_samples) {
            std::vector<pcl::PointXYZ> sampled_points;
            for (int i = 0; i <= num_samples; ++i) {
                float factor = (length / num_samples) * i;
                pcl::PointXYZ point;
                point.x = origin(0) + factor * direction(0);
                point.y = origin(1) + factor * direction(1);
                point.z = origin(2) + factor * direction(2);
                sampled_points.push_back(point);
            }
            return sampled_points;
        }

        Eigen::Vector3d transformPoint(const Eigen::Vector3d& point, const nav_msgs::Odometry& odom_msg) {
            Eigen::Vector3d position_ = Eigen::Vector3d(odom_msg.pose.pose.position.x,
                                                        odom_msg.pose.pose.position.y,
                                                        odom_msg.pose.pose.position.z);

            Eigen::Quaterniond orientation_ = Eigen::Quaterniond(odom_msg.pose.pose.orientation.w,
                                                                odom_msg.pose.pose.orientation.x,
                                                                odom_msg.pose.pose.orientation.y,
                                                                odom_msg.pose.pose.orientation.z);
    
            // 从四元数获取旋转矩阵
            Eigen::Matrix3d rot_matrix = orientation_.toRotationMatrix();
            // 使用旋转矩阵的逆对点进行变换，并加上位置信息
            Eigen::Vector3d transformed_point = rot_matrix * point + position_;
            return transformed_point;
        }

        void writePointsToFile(const std::string& file_path, const std::vector<Eigen::Vector3d>& pts_to_czd) {
            // 打开文件
            std::ofstream file(file_path);
            if (!file.is_open()) {
                std::cerr << "Failed to open the file: " << file_path << std::endl;
                return;
            }
            // 写入每个点
            for (const auto& pt : pts_to_czd) {
                file << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
            }
            // 关闭文件
            file.close();
        }

        void calculate_depth_and_position(const string camera) {
            
            auto start = std::chrono::high_resolution_clock::now();
            auto bbox_msg = bboxes_[camera];
            int lidar_idx = find_closest_diff(point_cloud_cache_, bbox_msg.header.stamp);
            int odom_idx = find_closest_diff(odometry_cache_, bbox_msg.header.stamp);
            if (lidar_idx == -1 || odom_idx == -1) return;
            sensor_msgs::PointCloud2 point_cloud_msg = point_cloud_cache_[lidar_idx];
            nav_msgs::Odometry odom_msg = odometry_cache_[odom_idx];
            

            // 用kd树查找最近点
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(point_cloud_msg, *cloud);
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud);
            
            size_t num_boxes = bbox_msg.bbox_xyxy.size() / 4;
            target_dist_calculator::DetectOut out_msg;
            out_msg.classes.resize(num_boxes);
            out_msg.global_poses.resize(num_boxes);
            
            Mat image_czd = images_[camera].clone();
            std::vector<Eigen::Vector3d> pts_to_czd;
            for (size_t i = 0; i < num_boxes; ++i) {
                // 解析边界框的坐标
                float x1 = bbox_msg.bbox_xyxy[4 * i];
                float y1 = bbox_msg.bbox_xyxy[4 * i + 1];
                float x2 = bbox_msg.bbox_xyxy[4 * i + 2];
                float y2 = bbox_msg.bbox_xyxy[4 * i + 3];
                float center_x = (x1 + x2) / 2.0f;
                float center_y = (y1 + y2) / 2.0f;

                // Convert image coordinates to camera coordinates (后续优化考虑畸变参数的)
                float center_x_camera = (center_x - camera_matrix_.at<double>(0, 2)) / camera_matrix_.at<double>(0, 0);
                float center_y_camera = (center_y - camera_matrix_.at<double>(1, 2)) / camera_matrix_.at<double>(1, 1);
                // std::cout<<"center_x_camera "<<center_x_camera<<" center_y_camera "<<center_y_camera<<std::endl;
                Eigen::Vector3d camera_origin_point = translation_vector_[camera];
                Eigen::Vector3d end_point_camera(center_x_camera, center_y_camera, 1.0);
                Eigen::Vector3d end_point_body = rotation_matrix_[camera] * end_point_camera + camera_origin_point; // camera to body frame
                Eigen::Vector3d bearing_camera = (end_point_body - camera_origin_point).normalized();
                std::array<Eigen::Vector3d, 2> line = {camera_origin_point, bearing_camera};
                // std::cout<<"end_point_body "<<end_point_body(0)<<" "<<end_point_body(1)<<" "<<end_point_body(2)<<std::endl;
                // publish_target_bearing(bearing_camera);

                auto distance_to_line = [](const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 2>& line) {
                    const auto& [A, direction_vector] = line;
                    Eigen::Vector3d B = A + direction_vector;
                    Eigen::Vector3d cross_product = (point - A).cross(B - A);
                    double distance = cross_product.norm() / (B - A).norm();
                    return distance;
                };

                // Find the closest point in the lidar data to the line
                Eigen::Vector3d closest_point;
                double min_distance = std::numeric_limits<double>::infinity();

                // 采样长度和采样点数
                float length = 15.0; // 修改为实际长度
                int num_samples = 50; // 修改为实际采样点数

                // 生成采样点
                std::vector<pcl::PointXYZ> sampled_points = samplePoints(camera_origin_point, bearing_camera, length, num_samples);
                
                // 查找每个采样点的最近邻
                for (const auto& sample_point : sampled_points) {
                    std::vector<int> pointIdxNKNSearch(1);
                    std::vector<float> pointNKNSquaredDistance(1);

                    if (kdtree.nearestKSearch(sample_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                        pcl::PointXYZ nearest_point = (*cloud)[pointIdxNKNSearch[0]];
                        float distance = sqrt(pointNKNSquaredDistance[0]);
                        if (distance < min_distance) {
                            min_distance = distance;
                            closest_point(0) = nearest_point.x;
                            closest_point(1) = nearest_point.y;
                            closest_point(2) = nearest_point.z;
                        }
                        // std::cout << "Sample Point: " << sample_point << " Nearest Point: " << nearest_point << " Distance: " << distance << std::endl;
                    }
                }

                

                // 发布最近点
                geometry_msgs::PointStamped point_stamped;
                point_stamped.header.stamp = ros::Time::now();
                point_stamped.header.frame_id = "body";
                point_stamped.point.x = closest_point.x();
                point_stamped.point.y = closest_point.y();
                point_stamped.point.z = closest_point.z();
                closest_point_pub_.publish(point_stamped);
                // 发布bearing
                publish_cam_line(camera_origin_point, bearing_camera, point_cloud_msg.header.stamp);


                // publish_cam_line(line[0], line[1], lidar_time);
                double depth = closest_point.norm();
                if ((min_distance <= dist_thre_final_) && (depth <= dist_use_lidar_)) {
                    Eigen::Vector3d closest_point_world = transformPoint(closest_point, odom_msg);
                    ROS_INFO("Use lidar Camera: %s, Depth: %.2f m, 3D Position in World: (%.2f, %.2f, %.2f), Min dist:%.2f",
                            camera.c_str(), depth, closest_point_world(0), closest_point_world(1), closest_point_world(2), min_distance);
                    std::cout<<(x2-x1)<<" "<<(y2-y1)<<" "<<depth<<std::endl;
                    out_msg.classes[i] = bbox_msg.bbox_cls[i];
                    out_msg.global_poses[i].x = closest_point_world(0);
                    out_msg.global_poses[i].y = closest_point_world(1);
                    out_msg.global_poses[i].z = closest_point_world(2);
                    
                    // 绘制操作端图片
                    // 创建深度和3D坐标的字符串
                    char depth_str[50];
                    char position_str[100];
                    sprintf(depth_str, "Depth: %.2f", depth);
                    sprintf(position_str, "3D Position: (%.2f, %.2f, %.2f)", closest_point_world(0), closest_point_world(1), closest_point_world(2));

                    // 在图像上绘制深度信息,3D坐标信息
                    // cv::putText(image_czd, depth_str, cv::Point(center_x, center_y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    // cv::putText(image_czd, position_str, cv::Point(center_x, center_y) + cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    pts_to_czd.push_back(closest_point_world);
                }
                // else {
                    {
                    if (y1<0.05) {
                        depth = person_width_ * width_rate_  / (x2-x1);
                    }
                    else {
                        depth = person_heidht_ * height_rate_  / (y2-y1);
                    }
                    closest_point = camera_origin_point +  depth*bearing_camera;
                    
                    point_stamped.header.stamp = ros::Time::now();
                    point_stamped.header.frame_id = "body";
                    point_stamped.point.x = closest_point.x();
                    point_stamped.point.y = closest_point.y();
                    point_stamped.point.z = closest_point.z();
                    closest_point_wh_pub_.publish(point_stamped);

                    Eigen::Vector3d closest_point_world2 = transformPoint(closest_point, odom_msg);
                    ROS_INFO("Use camera Camera: %s, Depth: %.2f m, 3D Position in World: (%.2f, %.2f, %.2f)",
                            camera.c_str(), depth, closest_point_world2(0), closest_point_world2(1), closest_point_world2(2));
                    // out_msg.classes[i] = bbox_msg.bbox_cls[i];
                    // out_msg.global_poses[i].x = closest_point_world(0);
                    // out_msg.global_poses[i].y = closest_point_world(1);
                    // out_msg.global_poses[i].z = closest_point_world(2);
                    
                    // pts_to_czd.push_back(closest_point_world);
                    // out_msg.classes[i] = -1;
                    // out_msg.global_poses[i].x = -1;
                    // out_msg.global_poses[i].y = -1;
                    // out_msg.global_poses[i].z = -1;
                    // pts_to_czd.push_back(Eigen::Vector3d(-1.0, -1.0, -1.0));
                }
            }
            // 可视化
            // 发布点云
            pc_pub_.publish(point_cloud_msg);
            
            // 发布结果话题
            out_msg.header.stamp = bbox_msg.header.stamp;
            out_msg.header.frame_id = "body";
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            cv_ptr->image = images_[camera];
            cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
            sensor_msgs::Image ros_image;
            cv_ptr->toImageMsg(ros_image);
            out_msg.image = ros_image;
            out_msg.image_camera = camera;
            out_info_pub_.publish(out_msg);

            // 保存操作端需要的图片
            imwrite(czd_img_save_path, image_czd);
            writePointsToFile(czd_save_path, pts_to_czd);
            bboxes_.clear();

            // 计算函数计算时间
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end - start;
            std::cout << "Duration: " << duration.count() << " seconds" << std::endl;

        }
        

        void imageCallback(const sensor_msgs::ImageConstPtr& msg, const string& camera) {
            std::cout<<"receive image"<<std::endl;
            // 图像回调函数
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            images_[camera] = cv_ptr->image;
            received_img_ = true;
        }

        void bboxCallback(const custom_msgs::BoundingBoxArrayConstPtr& msg, const string& camera) {
            std::cout<<"receive bbox"<<std::endl;
            // 目标框回调函数
            bboxes_[camera] = *msg;
            received_bbox_ = true;
            if (received_img_ && received_bbox_) {
                // 处理图像和目标框
                received_img_ = false;
                received_bbox_ = false;
                calculate_depth_and_position(camera);
            }
        }

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
            std::cout<<"receive lidar"<<std::endl;
            // 激光雷达点云回调函数
            if (point_cloud_cache_.size()>=20) point_cloud_cache_.pop_front();
            point_cloud_cache_.push_back(*msg);
            got_lidar_ = true;
        }

        void odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
            std::cout<<"receive odom"<<std::endl;
            // 里程计回调函数
            if (odometry_cache_.size()>=20) odometry_cache_.pop_front();
            odometry_cache_.push_back(*msg);
        }

    };

    int main(int argc, char** argv) {
        ros::init(argc, argv, "target_distance_calculator");
        ros::NodeHandle nh("~");
        TargetDistanceCalculator tdc(nh);
        ros::spin();
        return 0;
    }
