#include <fstream>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

struct GenericMessage {

  enum TYPE { IMU, POINTCLOUD, GNSS, IMAGE };

  TYPE msg_type;
  std::uint64_t timestamp_ns;
  std::shared_ptr<sensor_msgs::Imu> imu_msg;
  std::shared_ptr<sensor_msgs::PointCloud2> pointcloud_msg;
  std::shared_ptr<sensor_msgs::NavSatFix> gnss_msg;
  std::shared_ptr<sensor_msgs::Image> image_msg;

  friend std::ostream &operator<<(std::ostream &os, const GenericMessage &msg);
};

std::ostream &operator<<(std::ostream &os, const GenericMessage &msg) {
  switch (msg.msg_type) {
  case GenericMessage::TYPE::IMU:
    os << "IMU," << msg.timestamp_ns;
    break;
  case GenericMessage::TYPE::POINTCLOUD:
    os << "POINTCLOUD," << msg.timestamp_ns;
    break;
  case GenericMessage::TYPE::GNSS:
    os << "GNSS," << msg.timestamp_ns;
    break;
  case GenericMessage::TYPE::IMAGE:
    os << "IMAGE," << msg.timestamp_ns;
    break;
  }
  return os;
}

class GenericMessageContainer {

public:
  GenericMessageContainer() = default;

  void insert(const sensor_msgs::Imu &imu_msg) {
    GenericMessage generic_message;
    generic_message.imu_msg = std::make_shared<sensor_msgs::Imu>(imu_msg);
    generic_message.msg_type = GenericMessage::IMU;
    generic_message.timestamp_ns = imu_msg.header.stamp.toNSec();
    generic_message_vec.push_back(generic_message);
  }

  void insert(const sensor_msgs::PointCloud2 &pointcloud_msg) {
    GenericMessage generic_message;
    generic_message.pointcloud_msg = std::make_shared<sensor_msgs::PointCloud2>(pointcloud_msg);
    generic_message.msg_type = GenericMessage::POINTCLOUD;
    generic_message.timestamp_ns = pointcloud_msg.header.stamp.toNSec();
    generic_message_vec.push_back(generic_message);
  }

  void insert(const sensor_msgs::NavSatFix &gnss_msg) {
    GenericMessage generic_message;
    generic_message.gnss_msg = std::make_shared<sensor_msgs::NavSatFix>(gnss_msg);
    generic_message.msg_type = GenericMessage::GNSS;
    generic_message.timestamp_ns = gnss_msg.header.stamp.toNSec();
    generic_message_vec.push_back(generic_message);
  }

  void insert(const sensor_msgs::Image &image_msg) {
    GenericMessage generic_message;
    generic_message.image_msg = std::make_shared<sensor_msgs::Image>(image_msg);
    generic_message.msg_type = GenericMessage::IMAGE;
    generic_message.timestamp_ns = image_msg.header.stamp.toNSec();
    generic_message_vec.push_back(generic_message);
  }

  void sort() {
    std::sort(generic_message_vec.begin(), generic_message_vec.end(),
              [](const GenericMessage &a, const GenericMessage &b) {
                return a.timestamp_ns < b.timestamp_ns;
              });
  }

  using Iterator = typename std::vector<GenericMessage>::const_iterator;

  Iterator begin() const { return generic_message_vec.begin(); }

  Iterator end() const { return generic_message_vec.end(); }

private:
  std::vector<GenericMessage> generic_message_vec;
};