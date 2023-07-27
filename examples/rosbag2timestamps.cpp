#include "BagManager.hpp"
#include "GenericMessageContainer.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {

  std::cout << "argc: " << argc << std::endl;

  if (argc < 2) {
    std::cout << "usage: rosrun rosbag_utils <bag_file_path> <path_to_save>" << std::endl;
    return -1;
  }

  // get bag file path
  std::string bag_file_path(argv[1]);
  if (!std::experimental::filesystem::exists(bag_file_path))
  {
    std::cout << "bag_file_path: " << bag_file_path << " does not exist" << std::endl;
    return -1;
  }
  std::cout << "bag_file_path: " << bag_file_path << std::endl;

  // get directory to save timestamp.csv
  std::string path_to_save(argv[2]);
  if (path_to_save.back() == '/') {
    path_to_save.pop_back();
  }
  if (!std::experimental::filesystem::exists(path_to_save))
  {
    std::cout << "path_to_save: " << path_to_save << " does not exist" << std::endl;
    std::cout << "creating directory: " << path_to_save << std::endl;
    std::experimental::filesystem::create_directory(path_to_save);
  }
  std::cout << "path_to_save: " << path_to_save << std::endl;

  // create BagManager
  BagManager bag1(bag_file_path);
  bag1.printInfo();

  // get imu and pointcloud2 messages, change topic name if necessary
  std::vector<sensor_msgs::Imu> imu_msgs;
  bag1.getMessages(imu_msgs, "/imu");
  std::cout << "imu_msgs.size(): " << imu_msgs.size() << std::endl;

  std::vector<sensor_msgs::PointCloud2> pc2_msgs;
  bag1.getMessages(pc2_msgs, "/livox/lidar");
  std::cout << "pc2_msgs.size(): " << pc2_msgs.size() << std::endl;

  // add imu and pointcloud2 messages to GenericMessageContainer
  GenericMessageContainer gmc;
  for (const auto &imu_msg : imu_msgs) {
    gmc.insert(imu_msg);
  }
  for (const auto &pc2_msg : pc2_msgs) {
    gmc.insert(pc2_msg);
  }
  gmc.sort();

  // save timestamp.csv
  std::string timestamp_csv_path(path_to_save + "/timestamp.csv");
  std::ofstream timestamp_csv_file(timestamp_csv_path);
  timestamp_csv_file << std::fixed << std::setprecision(9);
  timestamp_csv_file << "# sensor, timestamp[ns]" << std::endl;
  for (const auto &generic_message : gmc) {
    timestamp_csv_file << generic_message << std::endl;
  }
  timestamp_csv_file.close();

  return 0;
}