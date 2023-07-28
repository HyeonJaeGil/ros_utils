#include "BagManager.hpp"
#include "GenericMessageContainer.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

using std::cout;
using std::endl;

int main(int argc, char **argv) {

  const std::string imu_topic("/imu");
  const std::string gnss_topic("/ublox_driver/receiver_lla");
  const std::string lidar_topic("/livox/lidar");

  cout << "\033[32mThis is an example of how to save IMU, GNSS, and LiDAR "
          "messages from a rosbag to files.\n"
       << "Please set appropriate topic names in the code.\033[0m\n"
       << "\033[32mCurrent IMU topic name: \033[1m" << imu_topic.c_str()
       << "\033[0m\n"
       << "\033[32mCurrent GNSS topic name: \033[1m" << gnss_topic.c_str()
       << "\033[0m\n"
       << "\033[32mCurrent LiDAR topic name: \033[1m" << lidar_topic.c_str()
       << "\033[0m\n"
       << endl;

  if (argc < 3) {
    cout << "usage: rosrun rosbag_utils rosbag2timestamps "
         << "<bag_file_path> <path_to_save>" << endl;
    return -1;
  }

  // get bag file path
  std::string bag_file_path(argv[1]);
  if (!std::experimental::filesystem::exists(bag_file_path)) {
    cout << "bag_file_path: " << bag_file_path << " does not exist" << endl;
    return -1;
  }
  cout << "bag_file_path: " << bag_file_path << endl;

  // get directory to save timestamp.csv
  std::string path_to_save(argv[2]);
  if (path_to_save.back() == '/') {
    path_to_save.pop_back();
  }
  if (!std::experimental::filesystem::exists(path_to_save)) {
    cout << "path_to_save: " << path_to_save << " does not exist" << endl;
    cout << "creating directory: " << path_to_save << endl;
    std::experimental::filesystem::create_directory(path_to_save);
  }
  cout << "path_to_save: " << path_to_save << endl;

  // create BagManager
  BagManager bag1(bag_file_path);
  bag1.printInfo();

  // get imu messages, change topic name if necessary
  std::vector<sensor_msgs::Imu> imu_msgs;
  bag1.getMessages(imu_msgs, imu_topic);
  cout << "imu_msgs.size(): " << imu_msgs.size() << endl;

  // get pointcloud2 messages, change topic name if necessary
  std::vector<sensor_msgs::PointCloud2> pc2_msgs;
  bag1.getMessages(pc2_msgs, lidar_topic);
  cout << "pc2_msgs.size(): " << pc2_msgs.size() << endl;

  // get navsatfix messages, change topic name if necessary
  std::vector<sensor_msgs::NavSatFix> navsatfix_msgs;
  bag1.getMessages(navsatfix_msgs, gnss_topic);
  {
    // subtract 18 seconds from the timestamp of each navsatfix message
    // this is only the case for ublox_driver/receiver_lla topic
    for (auto &navsatfix_msg : navsatfix_msgs)
      navsatfix_msg.header.stamp -= ros::Duration(18.0);
  }
  cout << "navsatfix_msgs.size(): " << navsatfix_msgs.size() << endl;

  // add imu, pointcloud2, and navsatfix messages to GenericMessageContainer
  GenericMessageContainer gmc;
  for (const auto &imu_msg : imu_msgs) {
    gmc.insert(imu_msg);
  }
  for (const auto &pc2_msg : pc2_msgs) {
    gmc.insert(pc2_msg);
  }
  for (const auto &navsatfix_msg : navsatfix_msgs) {
    gmc.insert(navsatfix_msg);
  }
  gmc.sort();

  // save timestamp.csv
  std::string timestamp_csv_path(path_to_save + "/timestamp.csv");
  std::ofstream timestamp_csv_file(timestamp_csv_path);
  timestamp_csv_file << std::fixed << std::setprecision(9);
  timestamp_csv_file << "# sensor, timestamp[ns]" << endl;
  for (const auto &generic_message : gmc) {
    timestamp_csv_file << generic_message << endl;
  }
  timestamp_csv_file.close();

  // save imu.csv
  std::string imu_csv_path(path_to_save + "/imu.csv");
  std::ofstream imu_csv_file(imu_csv_path);
  imu_csv_file << std::fixed << std::setprecision(9);
  imu_csv_file << "# timestamp[ns], wx, wy, wz, ax, ay, az" << endl;
  for (const auto &generic_message : gmc) {
    if (generic_message.msg_type == GenericMessage::IMU) {
      std::shared_ptr<sensor_msgs::Imu> imu_msg = generic_message.imu_msg;
      imu_csv_file << generic_message.timestamp_ns << ", ";
      imu_csv_file << imu_msg->angular_velocity.x << ", ";
      imu_csv_file << imu_msg->angular_velocity.y << ", ";
      imu_csv_file << imu_msg->angular_velocity.z << ", ";
      imu_csv_file << imu_msg->linear_acceleration.x << ", ";
      imu_csv_file << imu_msg->linear_acceleration.y << ", ";
      imu_csv_file << imu_msg->linear_acceleration.z << endl;
    }
  }
  imu_csv_file.close();

  // save gnss.csv
  std::string gnss_csv_path(path_to_save + "/gnss.csv");
  std::ofstream gnss_csv_file(gnss_csv_path);
  gnss_csv_file << std::fixed << std::setprecision(9);
  gnss_csv_file << "# timestamp[ns], latitude[deg], longitude[deg], altitude[m]"
                << endl;
  for (const auto &generic_message : gmc) {
    if (generic_message.msg_type == GenericMessage::GNSS) {
      std::shared_ptr<sensor_msgs::NavSatFix> gnss_msg =
          generic_message.gnss_msg;
      gnss_csv_file << generic_message.timestamp_ns << ", ";
      gnss_csv_file << gnss_msg->latitude << ", ";
      gnss_csv_file << gnss_msg->longitude << ", ";
      gnss_csv_file << gnss_msg->altitude << endl;
    }
  }
  gnss_csv_file.close();

  return 0;
}