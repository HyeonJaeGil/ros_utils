#pragma once
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

class TopicHelper {
public:
  TopicHelper() = delete;

  /**
   * @brief Convert ROS Image to OpenCV Image
   * 
   * @tparam T type of ROS Image (sensor_msgs::Image, sensor_msgs::CompressedImage, etc.)
   * @param msg ROS Image
   * @param encoding ROS Image encoding (e.g. "bgr8", "mono8", etc.)
   * @return cv::Mat OpenCV Image
   */
  template <typename T>
  static cv::Mat rosImage2cvImage(const T &msg, const std::string &encoding = std::string()) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, encoding);
    return cv_ptr->image;
  }

  /**
   * @brief Convert ROS CameraInfo to OpenCV CameraInfo
   * 
   * @param msg ROS CameraInfo
   * @param camera_matrix Output OpenCV Camera Matrix (3x3)
   * @param distortion_coefficients Output OpenCV Distortion Coefficients (1x5)
   */
  static void rosCameraInfo2cvCameraInfo(const sensor_msgs::CameraInfo &msg, cv::Mat &camera_matrix,
                                         cv::Mat &distortion_coefficients) {
    camera_matrix = cv::Mat(3, 3, CV_64FC1, const_cast<double *>(msg.K.data())).clone();
    distortion_coefficients = cv::Mat(1, 5, CV_64FC1, const_cast<double *>(msg.D.data())).clone();
  }

  // static void rosPointCloud2PCLPointCloud(const sensor_msgs::PointCloud2 &msg,
  //                                         pcl::PCLPointCloud2 &pcl_pc2);

  // static void rosPointCloud2PCLPointCloud(const sensor_msgs::PointCloud2 &msg,
  //                                         pcl::PointCloud<pcl::PointXYZI> &pcl_pc2);

  // static void rosPointCloud2pcd(const sensor_msgs::PointCloud2 &msg, std::string &pcd_path);

  // static void rosPointCloud2bin(const sensor_msgs::PointCloud2 &msg, std::string &bin_path);
};