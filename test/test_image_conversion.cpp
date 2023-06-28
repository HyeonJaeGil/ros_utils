#include "BagManager.hpp"
#include "TopicHelper.hpp"
#include <fstream>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

int main() {

  BagManager bag1("/home/hj/d435_2023-06-27-20-59-08.bag");
  bag1.printInfo();

  std::string save_path("/home/hj/" + bag1.getBagFileName());

  std::vector<sensor_msgs::Imu> imu_msgs;
  bag1.getMessages(imu_msgs, "/camera/imu");
  std::cout << "imu_msgs.size(): " << imu_msgs.size() << std::endl;

  std::string imu_csv_path(save_path + "/imu.csv");
  std::ofstream imu_csv_file;
  imu_csv_file.open(imu_csv_path);
  imu_csv_file << "timestamp,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,"
                  "angular_velocity_x,angular_velocity_y,angular_velocity_z"
               << std::endl;
  imu_csv_file << std::fixed << std::setprecision(9);
  for (const auto &imu_msg : imu_msgs) {
    imu_csv_file << imu_msg.header.stamp.toSec() << "," << imu_msg.linear_acceleration.x << ","
                 << imu_msg.linear_acceleration.y << "," << imu_msg.linear_acceleration.z << ","
                 << imu_msg.angular_velocity.x << "," << imu_msg.angular_velocity.y << ","
                 << imu_msg.angular_velocity.z << std::endl;
  }
  imu_csv_file.close();

  std::vector<sensor_msgs::Image> image_msgs;
  bag1.getMessages(image_msgs, "/camera/color/image_raw");
  std::cout << "image_msgs.size(): " << image_msgs.size() << std::endl;

  std::string image_png_path_prefix(save_path + "/rgb");
  for (int i = 0; i < image_msgs.size(); i++) {
    cv::Mat image = TopicHelper::rosImage2cvImage(image_msgs[i], "bgr8");
    std::string timestamp = std::to_string(image_msgs[i].header.stamp.toNSec());
    cv::imwrite(image_png_path_prefix + "/" + timestamp + ".png", image);
    cv::imshow("image", image);
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  std::vector<sensor_msgs::CameraInfo> camera_info_msgs;
  bag1.getMessages(camera_info_msgs, "/camera/color/camera_info");
  cv::Mat camera_info, dist_coeffs;
  TopicHelper::rosCameraInfo2cvCameraInfo(camera_info_msgs[0], camera_info, dist_coeffs);
  std::cout << "camera_info:\n" << camera_info << std::endl;
  std::cout << "dist_coeffs:\n" << dist_coeffs << std::endl;

  return 0;
}