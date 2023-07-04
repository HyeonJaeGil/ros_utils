#include "BagManager.hpp"
#include <fstream>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

int main() {

  std::string bag1_path("/home/hj/d435_2023-06-27-20-59-08.bag");

  BagManager bag1(bag1_path);
  bag1.printInfo();

  std::vector<sensor_msgs::Imu> imu_msgs;
  bag1.getMessages(imu_msgs, "/camera/imu");
  std::cout << "imu_msgs.size(): " << imu_msgs.size() << std::endl;

  std::vector<sensor_msgs::Image> image_msgs;
  bag1.getMessages(image_msgs, "/camera/color/image_raw");
  std::cout << "image_msgs.size(): " << image_msgs.size() << std::endl;

  // for each imu message, save its timestamp, linear_acceleration, angular_velocity to a csv file
  std::ofstream imu_csv_file;
  imu_csv_file.open("/home/hj/imu.csv");
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

  return 0;
}