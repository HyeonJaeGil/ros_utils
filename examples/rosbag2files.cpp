#include "BagManager.hpp"
#include "GenericMessageContainer.hpp"
#include "TopicHelper.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

int main() {

  BagManager bag1("/home/hj/Downloads/1030_shape_s.bag");
  bag1.printInfo();

  std::string save_path("/home/hj/" + bag1.getBagFileName());
  std::experimental::filesystem::create_directory(save_path);

  std::vector<sensor_msgs::Imu> imu_msgs;
  bag1.getMessages(imu_msgs, "/imu1");
  std::cout << "imu_msgs.size(): " << imu_msgs.size() << std::endl;

  std::vector<sensor_msgs::NavSatFix> navsatfix_msgs;
  bag1.getMessages(navsatfix_msgs, "/ublox_driver/receiver_lla");
  {
    // subtract 18 seconds from the timestamp of each navsatfix message
    // this is only the case for ublox_driver/receiver_lla topic
    for (auto &navsatfix_msg : navsatfix_msgs)
      navsatfix_msg.header.stamp -= ros::Duration(18.0);
  }
  std::cout << "navsatfix_msgs.size(): " << navsatfix_msgs.size() << std::endl;

  std::vector<sensor_msgs::PointCloud2> pc2_msgs;
  bag1.getMessages(pc2_msgs, "/velodyne_points");
  std::cout << "pc2_msgs.size(): " << pc2_msgs.size() << std::endl;

  GenericMessageContainer gmc;
  for (const auto &imu_msg : imu_msgs) {
    gmc.insert(imu_msg);
  }
  for (const auto &navsatfix_msg : navsatfix_msgs) {
    gmc.insert(navsatfix_msg);
  }
  for (const auto &pc2_msg : pc2_msgs) {
    gmc.insert(pc2_msg);
  }
  gmc.sort();

  // iterate and print each message
  for (const auto &generic_message : gmc) {
    std::cout << generic_message << std::endl;
  }

  // save timestamp.csv
  std::string timestamp_csv_path(save_path + "/timestamp.csv");
  std::ofstream timestamp_csv_file(timestamp_csv_path);
  timestamp_csv_file << std::fixed << std::setprecision(9);
  timestamp_csv_file << "# sensor, timestamp[ns]" << std::endl;
  for (const auto &generic_message : gmc) {
    timestamp_csv_file << generic_message << std::endl;
  }
  timestamp_csv_file.close();

  // save imu.csv
  std::string imu_csv_path(save_path + "/imu.csv");
  std::ofstream imu_csv_file(imu_csv_path);
  imu_csv_file << std::fixed << std::setprecision(9);
  imu_csv_file << "# timestamp[ns], wx, wy, wz, ax, ay, az" << std::endl;
  for (const auto &generic_message : gmc) {
    if (generic_message.msg_type == GenericMessage::IMU) {
      std::shared_ptr<sensor_msgs::Imu> imu_msg = generic_message.imu_msg;
      imu_csv_file << generic_message.timestamp_ns << ", ";
      imu_csv_file << imu_msg->angular_velocity.x << ", ";
      imu_csv_file << imu_msg->angular_velocity.y << ", ";
      imu_csv_file << imu_msg->angular_velocity.z << ", ";
      imu_csv_file << imu_msg->linear_acceleration.x << ", ";
      imu_csv_file << imu_msg->linear_acceleration.y << ", ";
      imu_csv_file << imu_msg->linear_acceleration.z << std::endl;
    }
  }

  // save gnss.csv
  std::string gnss_csv_path(save_path + "/gnss.csv");
  std::ofstream gnss_csv_file(gnss_csv_path);
  gnss_csv_file << std::fixed << std::setprecision(9);
  gnss_csv_file << "# timestamp[ns], latitude[deg], longitude[deg], altitude[m]" << std::endl;
  for (const auto &generic_message : gmc) {
    if (generic_message.msg_type == GenericMessage::GNSS) {
      std::shared_ptr<sensor_msgs::NavSatFix> gnss_msg = generic_message.gnss_msg;
      gnss_csv_file << generic_message.timestamp_ns << ", ";
      gnss_csv_file << gnss_msg->latitude << ", ";
      gnss_csv_file << gnss_msg->longitude << ", ";
      gnss_csv_file << gnss_msg->altitude << std::endl;
    }
  }

  return 0;
}