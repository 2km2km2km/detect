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
    #include <std_msgs/Float64MultiArray.h>
    #include <visualization_msgs/Marker.h>
    #include <visualization_msgs/MarkerArray.h>  

    #include <Eigen/Dense>
    #include <fstream>
    #include <cmath>
    #include <deque>
    #include <chrono>

    #include <iostream>
    #include <vector>
    #include <pcl/io/pcd_io.h>
    #include <pcl/common/common.h>
    #include <pcl/filters/extract_indices.h>
    #include <pcl/segmentation/extract_clusters.h>
    #include <pcl/visualization/cloud_viewer.h>
    #include <yaml-cpp/yaml.h>
    #include <tf2/LinearMath/Quaternion.h>
    #include <tf2/LinearMath/Transform.h>

    #include <pcl/common/transforms.h>
    #include <pthread.h>
    #include <sched.h>

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
            nh_.param("min_bbox_confi", min_bbox_confi_, 0.7);
            nh_.param("person_width", person_width_, 0.0);
            nh_.param("person_height", person_height_, 0.0);
            nh_.param("width_rate", width_rate_, 0.0);
            nh_.param("height_rate", height_rate_, 0.0);
            nh_.param("config_path", config_path,std::string("config_path"));
            nh_.param("czd_save_path", czd_save_path,std::string("czd_save_path"));
            nh_.param("czd_img_save_path", czd_img_save_path,std::string("czd_img_save_path"));
            nh_.param("pc_topic", pc_topic,std::string("pc_topic"));
            nh_.param("min_clu_number", min_clu_number_, 5);
            nh_.param("min_clu_dist", min_clu_dist_, 0.5);
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
            pos_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("pos_cloud", 10);
            clu_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("clu_cloud", 10);
            out_info_pub_ = nh_.advertise<target_dist_calculator::DetectOut>("detect_out", 10);
            depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth_image_topic", 10);
            image_pub_ = nh_.advertise<sensor_msgs::Image>("anno_image_topic", 10);
            clu_image_pub_ = nh_.advertise<sensor_msgs::Image>("clu_image_topic", 10);
            vis_clu_bbox_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vis_clu_bbox", 1);

        }

    private:
        ros::NodeHandle nh_;
        map<string, ros::Subscriber> image_subs_;
        map<string, ros::Subscriber> bbox_subs_;
        ros::Subscriber lidar_sub_, odometry_sub_;

        map<string, ros::Publisher> target_pubs_coord_;
        map<string, ros::Publisher> target_pubs_img_with_coord_;
        vector<ros::Publisher> target_pubs_coord_world_;
        ros::Publisher target_pub_bearing_camera_, depth_pub_, image_pub_, clu_image_pub_, vis_clu_bbox_pub_;
        ros::Publisher cam_line_pub_, closest_point_pub_, closest_point_wh_pub_, pc_pub_, pos_cloud_pub_, clu_cloud_pub_, out_info_pub_;

        int camera_type_;
        bool publish_raw_;
        double dist_thre_add_point_, dist_thre_final_, dist_use_lidar_;
        double person_width_, person_height_, width_rate_, height_rate_, min_bbox_confi_;
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

        int min_clu_number_;
        double min_clu_dist_;
        
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

        pcl::PointCloud<pcl::PointXYZ> transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_pointcloud, const nav_msgs::Odometry& odom_msg) {
            // 提取位置和平移信息
            double tx = odom_msg.pose.pose.position.x;
            double ty = odom_msg.pose.pose.position.y;
            double tz = odom_msg.pose.pose.position.z;

            tf2::Quaternion q(
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w
            );
            tf2::Matrix3x3 mat(q);

            // 构建4x4变换矩阵
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform(0, 0) = mat[0][0];
            transform(0, 1) = mat[0][1];
            transform(0, 2) = mat[0][2];
            transform(1, 0) = mat[1][0];
            transform(1, 1) = mat[1][1];
            transform(1, 2) = mat[1][2];
            transform(2, 0) = mat[2][0];
            transform(2, 1) = mat[2][1];
            transform(2, 2) = mat[2][2];
            transform(0, 3) = tx;
            transform(1, 3) = ty;
            transform(2, 3) = tz;

            // 创建一个输出点云对象
            pcl::PointCloud<pcl::PointXYZ> output_pointcloud;

            // 应用变换矩阵
            pcl::transformPointCloud(input_pointcloud, output_pointcloud, transform);

            // 返回转换后的点云
            return output_pointcloud;
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
            pcl::PointCloud<pcl::PointXYZ> pc_world = transformPointCloud(*cloud, odom_msg);
            // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            // kdtree.setInputCloud(cloud);


            // 参数配置
            int kernel_size = 3;  // 扩展点在深度图中的范围（3x3区域）
            double depth_scale = 255.0;  // 用于深度到颜色的映射
            double max_depth_value = 10.0;  // 假设最大深度为10米

            // 初始化深度图，使用float类型
            int width = 640;  // 假设640x480的深度图分辨率
            int height = 480;
            cv::Mat depth_image(height, width, CV_32FC1, cv::Scalar(0));
            Eigen::Matrix3d rotation_inv = rotation_matrix_[camera].transpose(); // 旋转矩阵的逆等于其转置
            Eigen::Vector3d translation_inv = -rotation_inv * translation_vector_[camera];

            // 遍历点云并生成深度图
            for (const auto& point : cloud->points) {
                // 将点从点云坐标系转换到相机坐标系
                Eigen::Vector3d point_in_camera = rotation_inv * Eigen::Vector3d(point.x, point.y, point.z) + translation_inv;

                double u = (camera_matrix_.at<double>(0, 0) * point_in_camera.x() / point_in_camera.z()) + camera_matrix_.at<double>(0, 2);
                double v = (camera_matrix_.at<double>(1, 1) * point_in_camera.y() / point_in_camera.z()) + camera_matrix_.at<double>(1, 2);

                // 检查(u, v)是否在图像范围内
                if (u >= 0 && u < width && v >= 0 && v < height) {
                    int new_u = static_cast<int>(u);
                    int new_v = static_cast<int>(v);
                    if (new_u >= 0 && new_u < width && new_v >= 0 && new_v < height) {
                        float& current_depth = depth_image.at<float>(new_v, new_u);
                        if (current_depth == 0 || current_depth > point_in_camera.z()) {
                            current_depth = static_cast<float>(point_in_camera.z());
                        }
                    }
                    // 为了可视化,拓展深度点
                    // for (int i = -kernel_size / 2; i <= kernel_size / 2; ++i) {
                    //     for (int j = -kernel_size / 2; j <= kernel_size / 2; ++j) {
                    //         int new_u = static_cast<int>(u) + i;
                    //         int new_v = static_cast<int>(v) + j;
                    //         if (new_u >= 0 && new_u < width && new_v >= 0 && new_v < height) {
                    //             float& current_depth = depth_image.at<float>(new_v, new_u);
                    //             if (current_depth == 0 || current_depth > point_in_camera.z()) {
                    //                 current_depth = static_cast<float>(point_in_camera.z());
                    //             }
                    //         }
                    //     }
                    // }
                }
            }
            
            // 将深度图转换为颜色图用于可视化
            cv::Mat depth_image_8u;
            depth_image.convertTo(depth_image_8u, CV_8UC1, depth_scale / max_depth_value);
            cv::Mat color_mapped_image;
            cv::applyColorMap(depth_image_8u, color_mapped_image, cv::COLORMAP_JET);

            cv::Mat vis_cluster = cv::Mat::zeros(color_mapped_image.size(), color_mapped_image.type());

            Mat pub_image = images_[camera].clone();
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_image).toImageMsg();
            image_pub_.publish(img_msg);

            size_t num_boxes = bbox_msg.bbox_xyxy.size() / 4;
            for (size_t i = 0; i < num_boxes; ++i) {
                if (bbox_msg.bbox_conf[i]<min_bbox_confi_) num_boxes--;
            }
            target_dist_calculator::DetectOut out_msg;
            out_msg.classes.resize(num_boxes);
            out_msg.global_poses.resize(num_boxes);
            out_msg.bbox_min.resize(num_boxes);
            out_msg.bbox_max.resize(num_boxes);
            
            Mat image_czd = images_[camera].clone();
            std::vector<Eigen::Vector3d> pts_to_czd;

            pcl::PointCloud<pcl::PointXYZ>::Ptr pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointCloud<pcl::PointXYZ>::Ptr clu_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector3d min_bbox_world(-1,-1,-1), max_bbox_world(-1,-1,-1);
            visualization_msgs::MarkerArray vis_clu_bboxs;
            for (size_t i = 0; i < num_boxes; ++i) {
                if (bbox_msg.bbox_conf[i]<min_bbox_confi_) continue;
                // 解析边界框的坐标
                float x1 = bbox_msg.bbox_xyxy[4 * i];
                float y1 = bbox_msg.bbox_xyxy[4 * i + 1];
                float x2 = bbox_msg.bbox_xyxy[4 * i + 2];
                float y2 = bbox_msg.bbox_xyxy[4 * i + 3];
                float center_x = (x1 + x2) / 2.0f;
                float center_y = (y1 + y2) / 2.0f;
                // std::cout<<"bbox_msg confi"<<bbox_msg.bbox_conf[i]<<std::endl;
                // Convert image coordinates to camera coordinates (后续优化考虑畸变参数的)
                float center_x_camera = (center_x - camera_matrix_.at<double>(0, 2)) / camera_matrix_.at<double>(0, 0);
                float center_y_camera = (center_y - camera_matrix_.at<double>(1, 2)) / camera_matrix_.at<double>(1, 1);
                // std::cout<<"center_x_camera "<<center_x_camera<<" center_y_camera "<<center_y_camera<<std::endl;
                Eigen::Vector3d camera_origin_point = translation_vector_[camera];
                Eigen::Vector3d end_point_camera(center_x_camera, center_y_camera, 1.0);
                Eigen::Vector3d end_point_body = rotation_matrix_[camera] * end_point_camera + camera_origin_point; // camera to body frame
                Eigen::Vector3d bearing_camera = (end_point_body - camera_origin_point).normalized();

                cv::Point pt1(x1, y1);  // 左上角
                cv::Point pt2(x2, y2);  // 右下角

                // 矩形框的颜色和厚度
                cv::Scalar color(0, 255, 0);  // 绿色
                int thickness = 5;  // 线条厚度
                // std::cout<<"rect"<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<std::endl;
                // 在图像上绘制矩形框
                cv::rectangle(color_mapped_image, pt1, pt2, color, thickness);

                // 初始化点云
                pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                for (int v = y1; v < y2; ++v) {
                    for (int u = x1; u < x2; ++u) {
                        float depth = depth_image.at<float>(v, u);
                        if (depth > 0) {  // 忽略深度值为0的点
                            // 将(u, v, depth)转换到相机坐标系
                            Eigen::Vector3d point_in_camera;
                            point_in_camera.x() = (u - camera_matrix_.at<double>(0, 2)) * depth / camera_matrix_.at<double>(0, 0);
                            point_in_camera.y() = (v - camera_matrix_.at<double>(1, 2)) * depth / camera_matrix_.at<double>(1, 1);
                            point_in_camera.z() = depth;
                            Eigen::Vector3d point_in_body = rotation_matrix_[camera] * point_in_camera + translation_vector_[camera];
                            // 将三维点添加到点云中
                            pcl::PointXYZ pcl_point;
                            pcl_point.x = static_cast<float>(point_in_body.x());
                            pcl_point.y = static_cast<float>(point_in_body.y());
                            pcl_point.z = static_cast<float>(point_in_body.z());
                            bbox_cloud->points.push_back(pcl_point);
                        }
                    }
                }

                // 设置点云的宽和高
                bbox_cloud->width = static_cast<uint32_t>(bbox_cloud->points.size());
                bbox_cloud->height = 1;  // 由于是稀疏点云，设置为1行
                bbox_cloud->is_dense = false;  // 点云不是稠密的
                std::cout<<"bbox_cloud->points.size() "<<bbox_cloud->points.size()<<std::endl;
                std::vector<pcl::PointIndices> cluster_indices;
                double depth;
                Eigen::Vector3d closest_point;
                geometry_msgs::PointStamped point_stamped;
                if (bbox_cloud->points.size() > 0 ){
                    // 创建 KD-Tree 对象用于搜索
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                    tree->setInputCloud(bbox_cloud);

                    // 设置欧式聚类提取器
                    
                    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                    ec.setClusterTolerance(min_clu_dist_); // 设置近邻搜索的距离阈值（单位：米）
                    ec.setMinClusterSize(min_clu_number_);    // 设置一个聚类需要的最少点数目
                    ec.setMaxClusterSize(25000);  // 设置一个聚类需要的最多点数目
                    ec.setSearchMethod(tree);
                    ec.setInputCloud(bbox_cloud);
                    ec.extract(cluster_indices);

                    std::vector<cv::Vec3b> cluster_colors = {
                        cv::Vec3b(0, 0, 255),   // 红色
                        cv::Vec3b(0, 255, 0),   // 绿色
                        cv::Vec3b(255, 0, 0),   // 蓝色
                        cv::Vec3b(255, 255, 0), // 黄色
                        cv::Vec3b(0, 255, 255)  // 青色
                    };
                    
                    
                    // std::cout<<"cluster_indices "<<cluster_indices.size()<<std::endl;
                    // std::cout<<"w "<<x2-x1<<" h "<<y2-y1<<std::endl;
                    if(cluster_indices.size()>0) {
                        // 遍历每个聚类
                        int cluster_id = 0;
                        double max_dispersion = -1.0;
                        double min_depth = std::numeric_limits<double>::infinity();
                        int best_cluster_id = -1; 
                        for (const auto& indices : cluster_indices) {
                            std::cout<<"cluster_id "<<cluster_id<<" size "<<indices.indices.size()<<std::endl;
                            // cv::Vec3b color = cluster_colors[cluster_id % cluster_colors.size()];  // 选择颜色
                            Eigen::Vector3d centroid(0, 0, 0);
                            // 遍历聚类中的每个点
                            for (const auto& idx : indices.indices) {
                                const auto& point = bbox_cloud->points[idx];
                                // clu_cloud->points.push_back(bbox_cloud->points[idx]);

                                centroid += Eigen::Vector3d(bbox_cloud->points[idx].x, bbox_cloud->points[idx].y, bbox_cloud->points[idx].z);
                                // 可视化
                                // // 将点从点云坐标系转换到相机坐标系
                                // Eigen::Vector3d point_in_camera = rotation_inv * Eigen::Vector3d(point.x, point.y, point.z) + translation_inv;
                                // // 将3D点投影到图像平面
                                // double u = (camera_matrix_.at<double>(0, 0) * point_in_camera.x() / point_in_camera.z()) + camera_matrix_.at<double>(0, 2);
                                // double v = (camera_matrix_.at<double>(1, 1) * point_in_camera.y() / point_in_camera.z()) + camera_matrix_.at<double>(1, 2);

                                // // 检查(u, v)是否在图像范围内
                                // if (u >= 0 && u < width && v >= 0 && v < height) {
                                //     // 对(u, v)附近的区域进行深度值扩展并上色
                                //     for (int i = -kernel_size / 2; i <= kernel_size / 2; ++i) {
                                //         for (int j = -kernel_size / 2; j <= kernel_size / 2; ++j) {
                                //             int new_u = static_cast<int>(u) + i;
                                //             int new_v = static_cast<int>(v) + j;
                                //             if (new_u >= 0 && new_u < width && new_v >= 0 && new_v < height) {
                                //                 float& current_depth = depth_image.at<float>(new_v, new_u);
                                //                 if (current_depth == 0 || current_depth > point_in_camera.z()) {
                                //                     current_depth = static_cast<float>(point_in_camera.z());
                                //                     vis_cluster.at<cv::Vec3b>(new_v, new_u) = color;  // 设置颜色
                                //                 }
                                //             }
                                //         }
                                //     }
                                // }
                            }
                            centroid /= static_cast<double>(indices.indices.size());
                            
                            
                            // // closest_point = centroid;
                            
                            Eigen::Vector3d centroid_in_camera = rotation_inv * centroid + translation_inv;
                            Eigen::Vector2d centroid_in_img;
                            centroid_in_img(0) = (camera_matrix_.at<double>(0, 0) * centroid_in_camera.x() / centroid_in_camera.z()) + camera_matrix_.at<double>(0, 2);
                            centroid_in_img(1) = (camera_matrix_.at<double>(1, 1) * centroid_in_camera.y() / centroid_in_camera.z()) + camera_matrix_.at<double>(1, 2);
                            // 质心是不是在图像中心
                            // std::cout<<"(centroid_in_img(1) - y1 / (y2 - y1) "<<((centroid_in_img(1) - y1) / (y2 - y1))<<std::endl;
                            // if (((centroid_in_img(1) - y1) / (y2 - y1)) > 0.2 && ((centroid_in_img(1) - y1) / (y2 - y1)) < 0.70) {
                            //     closest_point = centroid;
                            //     best_cluster_id = cluster_id;
                            //     // continue;
                            //     break;
                            // }
                            
                            
                            // 计算离散度（点到质心的平均距离）
                            // double dispersion = 0.0;
                            // for (const auto& idx : indices.indices) {
                            //     Eigen::Vector3d point(bbox_cloud->points[idx].x, bbox_cloud->points[idx].y, bbox_cloud->points[idx].z);
                            //     // dispersion += (point - centroid).norm();
                            //     // 算二维距离
                            //     Eigen::Vector3d point_in_camera = rotation_inv * point + translation_inv;
                            //     Eigen::Vector2d point_in_img;
                            //     point_in_img(0) = (camera_matrix_.at<double>(0, 0) * point_in_camera.x() / point_in_camera.z()) + camera_matrix_.at<double>(0, 2);
                            //     point_in_img(1) = (camera_matrix_.at<double>(1, 1) * point_in_camera.y() / point_in_camera.z()) + camera_matrix_.at<double>(1, 2);
                            //     dispersion += (point_in_img - centroid_in_img).norm();
                            // }
                            // dispersion /= indices.indices.size();
                            // // std::cout<<"dispersion "<<dispersion<<std::endl;

                            // // 找到离散度最大的聚类
                            // if ((dispersion > max_dispersion) && (((centroid_in_img(1) - y1) / (y2 - y1)) > 0.2 && ((centroid_in_img(1) - y1) / (y2 - y1)) < 0.70) ) {
                            //     max_dispersion = dispersion;
                            //     closest_point = centroid;
                            //     best_cluster_id = cluster_id;
                            // }
                            // 找到深度最近的点云
                            if ((centroid.norm() < min_depth) && (((centroid_in_img(1) - y1) / (y2 - y1)) > 0.2 && ((centroid_in_img(1) - y1) / (y2 - y1)) < 0.70) ) {
                                min_depth = centroid.norm();
                                closest_point = centroid;
                                best_cluster_id = cluster_id;
                            }

                            // 输出结果
                            // std::cout << "Cluster " << cluster_id << " centroid: ["
                            //         << centroid.x() << ", " << centroid.y() << ", " << centroid.z() << "]" << std::endl;
                            // std::cout<<"depth: "<<centroid.norm()<<std::endl;
                            // 增加 cluster_id，用于选择下一种颜色
                            cluster_id++;
                            // 不增加,只用第一类:
                            // break;
                        }
                        if(best_cluster_id != -1) {
                            //可视化
                            double min_x = std::numeric_limits<double>::infinity();
                            double min_y = std::numeric_limits<double>::infinity();
                            double min_z = std::numeric_limits<double>::infinity();
                            double max_x = -std::numeric_limits<double>::infinity();
                            double max_y = -std::numeric_limits<double>::infinity();
                            double max_z = -std::numeric_limits<double>::infinity();
                            for (const auto& idx : cluster_indices[best_cluster_id].indices) {
                                clu_cloud->points.push_back(bbox_cloud->points[idx]);
                                Eigen::Vector3d bpoint_world = transformPoint(Eigen::Vector3d(bbox_cloud->points[idx].x, bbox_cloud->points[idx].y, bbox_cloud->points[idx].z), odom_msg);
                                if (min_x > bpoint_world(0)) min_x = bpoint_world(0);
                                if (min_y > bpoint_world(1)) min_y = bpoint_world(1);
                                if (min_z > bpoint_world(2)) min_z = bpoint_world(2);
                                if (max_x < bpoint_world(0)) max_x = bpoint_world(0);
                                if (max_y < bpoint_world(1)) max_y = bpoint_world(1);
                                if (max_z < bpoint_world(2)) max_z = bpoint_world(2);
                            }
                            min_bbox_world = Eigen::Vector3d(min_x, min_y, min_z);
                            max_bbox_world = Eigen::Vector3d(max_x, max_y, max_z);
                            
                            visualization_msgs::Marker vis_clu_bbox;
                            vis_clu_bbox.header.frame_id = "world";
                            vis_clu_bbox.header.stamp = ros::Time::now();
                            vis_clu_bbox.ns = "basic_shapes";
                            vis_clu_bbox.id = i;  // 每个Marker需要一个唯一的ID
                            vis_clu_bbox.type = visualization_msgs::Marker::CUBE;
                            vis_clu_bbox.action = visualization_msgs::Marker::ADD;
                            vis_clu_bbox.pose.position.x = (max_bbox_world(0) + min_bbox_world(0))/2;
                            vis_clu_bbox.pose.position.y = (max_bbox_world(1) + min_bbox_world(1))/2;
                            vis_clu_bbox.pose.position.z = (max_bbox_world(2) + min_bbox_world(2))/2;
                            vis_clu_bbox.pose.orientation.x = 0.0;
                            vis_clu_bbox.pose.orientation.y = 0.0;
                            vis_clu_bbox.pose.orientation.z = 0.0;
                            vis_clu_bbox.pose.orientation.w = 1.0;
                            // 设置Marker的尺寸
                            vis_clu_bbox.scale.x = max_bbox_world(0) - min_bbox_world(0);
                            vis_clu_bbox.scale.y = max_bbox_world(1) - min_bbox_world(1);
                            vis_clu_bbox.scale.z = max_bbox_world(2) - min_bbox_world(2);
                            // 设置Marker的颜色
                            vis_clu_bbox.color.r = 1.0f - 0.2f * i;  // 不同的颜色
                            vis_clu_bbox.color.g = 0.2f * i;
                            vis_clu_bbox.color.b = 0.5f;
                            vis_clu_bbox.color.a = 0.2f;
                            vis_clu_bbox.lifetime = ros::Duration();  // 保持显示直到被覆盖
                            vis_clu_bboxs.markers.push_back(vis_clu_bbox);


                            // publish_cam_line(line[0], line[1], lidar_time);
                            depth = closest_point.norm();
                            std::cout<<"depth1 "<<depth<<std::endl;
                        }
                        else {
                            depth = std::numeric_limits<double>::infinity();
                        }

                    }
                }
                if ((bbox_cloud->points.size() > 0 ) && (cluster_indices.size()>0) && (depth <= dist_use_lidar_)) {
                    // std::cout<<"depth2 "<<depth<<std::endl;
                    // 使用深度加延长来确认位置
                    closest_point = camera_origin_point + depth * bearing_camera;
                    Eigen::Vector3d closest_point_world = transformPoint(closest_point, odom_msg);
                    ROS_INFO("Use lidar Camera: %s, Depth: %.2f m, 3D Position in World: (%.2f, %.2f, %.2f)",
                            camera.c_str(), depth, closest_point_world(0), closest_point_world(1), closest_point_world(2));
                    // std::cout<<(x2-x1)<<" "<<(y2-y1)<<" "<<depth<<std::endl;
                    out_msg.classes[i] = bbox_msg.bbox_cls[i];
                    out_msg.global_poses[i].x = closest_point_world(0);
                    out_msg.global_poses[i].y = closest_point_world(1);
                    out_msg.global_poses[i].z = closest_point_world(2);
                    out_msg.bbox_min[i].x = min_bbox_world(0);
                    out_msg.bbox_min[i].y = min_bbox_world(1);
                    out_msg.bbox_min[i].z = min_bbox_world(2);
                    out_msg.bbox_max[i].x = max_bbox_world(0);
                    out_msg.bbox_max[i].y = max_bbox_world(1);
                    out_msg.bbox_max[i].z = max_bbox_world(2);
                    // 绘制操作端图片
                    // 创建深度和3D坐标的字符串
                    char depth_str[50];
                    char position_str[100];
                    sprintf(depth_str, "Depth: %.2f", depth);
                    sprintf(position_str, "3D Position: (%.2f, %.2f, %.2f)", closest_point_world(0), closest_point_world(1), closest_point_world(2));
                    // 用点云估计人身高
                    person_width_ = depth * (x2-x1) / width_rate_;
                    // 限制不要太低
                    if ((depth * (y2-y1) / height_rate_) > 1.4 )person_height_ = depth * (y2-y1) / height_rate_;
                    
                    // std::cout<<"person_height_ "<<person_height_<<std::endl;
                    // 在图像上绘制深度信息,3D坐标信息
                    // cv::putText(image_czd, depth_str, cv::Point(center_x, center_y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    // cv::putText(image_czd, position_str, cv::Point(center_x, center_y) + cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    pts_to_czd.push_back(closest_point_world);
                    point_stamped.header.stamp = ros::Time::now();
                    point_stamped.header.frame_id = "world";
                    point_stamped.point.x = closest_point_world.x();
                    point_stamped.point.y = closest_point_world.y();
                    point_stamped.point.z = closest_point_world.z();
                    closest_point_wh_pub_.publish(point_stamped);
                    pos_cloud->points.push_back(pcl::PointXYZ(closest_point_world.x(), closest_point_world.y(), closest_point_world.z()));
                    // closest_point_pub_.publish(point_stamped);
                }
                else
                {
                    if (y1<0.05) {
                        depth = person_width_ * width_rate_  / (x2-x1);
                    }
                    else {
                        depth = person_height_ * height_rate_  / (y2-y1);
                    }
                    closest_point = camera_origin_point +  depth*bearing_camera;


                    std::cout<<"person_height_ "<<person_height_<<std::endl;
                    Eigen::Vector3d closest_point_world2 = transformPoint(closest_point, odom_msg);
                    ROS_INFO("Use camera Camera: %s, Depth: %.2f m, 3D Position in World: (%.2f, %.2f, %.2f)",
                            camera.c_str(), depth, closest_point_world2(0), closest_point_world2(1), closest_point_world2(2));
                    out_msg.classes[i] = bbox_msg.bbox_cls[i];
                    out_msg.global_poses[i].x = closest_point_world2(0);
                    out_msg.global_poses[i].y = closest_point_world2(1);
                    out_msg.global_poses[i].z = closest_point_world2(2);
                    out_msg.bbox_min[i].x = min_bbox_world(0);
                    out_msg.bbox_min[i].y = min_bbox_world(1);
                    out_msg.bbox_min[i].z = min_bbox_world(2);
                    out_msg.bbox_max[i].x = max_bbox_world(0);
                    out_msg.bbox_max[i].y = max_bbox_world(1);
                    out_msg.bbox_max[i].z = max_bbox_world(2);

                    pts_to_czd.push_back(closest_point_world2);

                    point_stamped.header.stamp = ros::Time::now();
                    point_stamped.header.frame_id = "world";
                    point_stamped.point.x = closest_point_world2.x();
                    point_stamped.point.y = closest_point_world2.y();
                    point_stamped.point.z = closest_point_world2.z();
                    closest_point_wh_pub_.publish(point_stamped);
                    pos_cloud->points.push_back(pcl::PointXYZ(closest_point_world2.x(), closest_point_world2.y(), closest_point_world2.z()));


                }
            }
            sensor_msgs::ImagePtr clu_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", vis_cluster).toImageMsg();
            clu_image_pub_.publish(clu_img_msg);
            // 将彩色图像转换为 ROS 消息并发布
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_mapped_image).toImageMsg();
            depth_pub_.publish(depth_msg);
            // 可视化
            sensor_msgs::PointCloud2 pc_world_msg;

            // 将PCL的点云转换为ROS的消息格式
            pcl::toROSMsg(pc_world, pc_world_msg);

            // 设置header
            pc_world_msg.header.frame_id = "world";  // 根据需要设置frame_id
            pc_world_msg.header.stamp = ros::Time::now();

            // 发布点云
            pc_pub_.publish(pc_world_msg);

            if(pos_cloud->points.size()>0){
                sensor_msgs::PointCloud2 pos_cloud_msg;
                pcl::toROSMsg(*pos_cloud, pos_cloud_msg);
                pos_cloud_msg.header.frame_id = "world";  // 根据需要设置frame_id
                pos_cloud_msg.header.stamp = ros::Time::now();
                pos_cloud_pub_.publish(pos_cloud_msg);
            }
            if(clu_cloud->points.size()>0){
                sensor_msgs::PointCloud2 clu_cloud_msg;
                pcl::PointCloud<pcl::PointXYZ> clu_cloud_world = transformPointCloud(*clu_cloud, odom_msg);
                pcl::toROSMsg(clu_cloud_world, clu_cloud_msg);
                clu_cloud_msg.header.frame_id = "world";  // 根据需要设置frame_id
                clu_cloud_msg.header.stamp = ros::Time::now();
                clu_cloud_pub_.publish(clu_cloud_msg);

            }

            if (num_boxes > 0) {
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
            }
            if (vis_clu_bboxs.markers.size()>0) vis_clu_bbox_pub_.publish(vis_clu_bboxs);



            // // 保存操作端需要的图片
            // imwrite(czd_img_save_path, image_czd);
            // writePointsToFile(czd_save_path, pts_to_czd);
            bboxes_.clear();

            // 计算函数计算时间
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end - start;
            std::cout << "Duration: " << duration.count() << " seconds" << std::endl;

        }
        

        void imageCallback(const sensor_msgs::ImageConstPtr& msg, const string& camera) {
            // std::cout<<"receive image"<<std::endl;
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
            // std::cout<<"receive bbox"<<std::endl;
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
            // std::cout<<"receive lidar"<<std::endl;
            // 激光雷达点云回调函数
            if (point_cloud_cache_.size()>=20) point_cloud_cache_.pop_front();
            point_cloud_cache_.push_back(*msg);
            got_lidar_ = true;
        }

        void odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
            // std::cout<<"receive odom"<<std::endl;
            // 里程计回调函数
            if (odometry_cache_.size()>=20) odometry_cache_.pop_front();
            odometry_cache_.push_back(*msg);
        }

    };
    void set_affinity(int cpu_id) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_id, &cpuset);

        pthread_t current_thread = pthread_self();    
        pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    }
    int main(int argc, char** argv) {
        set_affinity(0);
        ros::init(argc, argv, "target_distance_cluster_calculator");
        ros::NodeHandle nh("~");
        TargetDistanceCalculator tdc(nh);
        ros::spin();

        return 0;
    }
