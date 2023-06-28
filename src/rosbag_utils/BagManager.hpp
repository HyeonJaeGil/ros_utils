#pragma once
#include <iomanip>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

using std::cout;
using std::endl;
using std::left;
using std::right;
using std::setw;

class BagManager {
public:
  /**
   * @brief Construct a new Bag Manager object
   *
   * @param bag_in_path
   * @param bag_out_path
   */
  BagManager(std::string bag_in_path, std::string bag_out_path = std::string())
      : bag_in_path_(bag_in_path), bag_out_path_(bag_out_path) {
    if (bag_out_path_.empty()) { // append "new_" to the bag name
      bag_out_path_ = bag_in_path_;
      bag_out_path_.insert(bag_out_path_.find_last_of('.'), "_new");
    }
    bag_in_.open(bag_in_path, rosbag::BagMode::Read);
    bag_out_.open(bag_out_path_, rosbag::BagMode::Write);
  }

  /**
   * @brief Destroy the Bag Manager object
   *
   */
  ~BagManager() {
    bag_in_.close();
    bag_out_.close();
  }

  /**
   * @brief Get the Start Time object
   *
   * @return double beginning time of bag file [in seconds]
   */
  double getStartTime() const {
    rosbag::View view(bag_in_);
    return view.getBeginTime().toSec();
  }

  /**
   * @brief Get the End Time object
   *
   * @return double ending time of bag file [in seconds]
   */
  double getEndTime() const {
    rosbag::View view(bag_in_);
    return view.getEndTime().toSec();
  }

  /**
   * @brief Get the Bag File Name object
   *
   * @return std::string basename of bag file (without extension)
   */
  std::string getBagFileName() const {
    std::string bag_name = bag_in_path_.substr(bag_in_path_.find_last_of("/") + 1);
    bag_name = bag_name.substr(0, bag_name.find_last_of("."));
    return bag_name;
  }

  /**
   * @brief Get the Type object
   *
   * @param topic
   * @return std::string type of topic
   */
  std::string getType(std::string topic) const {
    rosbag::View view(bag_in_, rosbag::TopicQuery(topic));
    for (const auto &m : view) {
      return m.getDataType();
    }
    return std::string(); // return empty string if no message is found
  }

  /**
   * @brief Get the Topics object
   *
   * @return std::vector<std::string> vector of topics in bag file
   */
  std::vector<std::string> getTopics() const {
    std::set<std::string> topics_set;
    for (const auto &m : rosbag::View(bag_in_)) {
      topics_set.insert(m.getTopic());
    }
    std::vector<std::string> topics;
    for (const auto &topic : topics_set) {
      topics.push_back(topic);
    }
    topics_set.clear();
    return topics;
  }

  /**
   * @brief Get the Topics With Types object
   *
   * @return std::vector<std::pair<std::string, std::string>> vector of pairs of topic and type
   */
  std::vector<std::pair<std::string, std::string>> getTopicsWithTypes() const {
    std::vector<std::string> topics = getTopics();
    std::vector<std::pair<std::string, std::string>> topics_with_types;
    for (const auto &topic : topics) {
      topics_with_types.push_back(std::make_pair(topic, getType(topic)));
    }
    return topics_with_types;
  }

  /**
   * @brief Get the Messages object
   *
   * @tparam T type of message
   * @param msgs std::vector<T> vector of messages to be filled
   * @param topic std::string topic name
   */
  template <typename T> void getMessages(std::vector<T> &msgs, std::string topic) {
    rosbag::View view(bag_in_, rosbag::TopicQuery(topic));
    for (const auto &m : view) {
      msgs.push_back(*m.instantiate<T>());
    }
  }

  /**
   * @brief write a new bag file with only the specified topic name changed
   *
   * @param old_topic_name std::string old topic name
   * @param new_topic_name std::string new topic name
   */
  void changeTopicName(std::string old_topic_name, std::string new_topic_name) {
    for (const auto &m : rosbag::View(bag_in_)) {
      if (m.getTopic() == old_topic_name) {
        bag_out_.write(new_topic_name, m.getTime(), m, m.getConnectionHeader());
      } else {
        bag_out_.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
      }
    }
  }

  /**
   * @brief write a new bag file with only the frame id of specified topic name changed
   *
   * @param topic_name std::string topic name to be changed
   * @param new_frame_id std::string new frame id
   */
  void changeFrameIdOfTopic(std::string topic_name, std::string new_frame_id);

  /**
   * @brief split the bag file into two parts, based on the split time
   *
   * @param bag_out_path1 std::string bag file path of the first part [start, split_time]
   * @param bag_out_path2 std::string bag file path of the second part [split_time, end]
   * @param split_time double split time [in seconds]
   */
  void splitWithTime(std::string bag_out_path1, std::string bag_out_path2, double split_time) {
    rosbag::Bag bag_out1, bag_out2;
    bag_out1.open(bag_out_path1, rosbag::BagMode::Write);
    bag_out2.open(bag_out_path2, rosbag::BagMode::Write);
    // split the bag file into two parts, regardless of the topic
    for (const auto &m : rosbag::View(bag_in_)) {
      if (m.getTime().toSec() < split_time) {
        bag_out1.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
      } else {
        bag_out2.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
      }
    }
    bag_out1.close();
    bag_out2.close();
  }

  /**
   * @brief crop the bag file with the specified start and end time
   *
   * @param bag_out_path std::string bag file path of the cropped bag file
   * @param start_time start time [in seconds]
   * @param end_time end time [in seconds]
   */
  void cropWithTime(std::string bag_out_path, double start_time, double end_time);

  /**
   * @brief print the information of the bag file
   *
   */
  void printInfo() const {
    cout << std::fixed << std::setprecision(6);
    cout << "bagfile path: " << bag_in_path_ << endl;
    cout << setw(12) << left << "start time: " << setw(16) << left << getStartTime() << "[s]"
         << endl;
    cout << setw(12) << left << "end time: " << setw(16) << left << getEndTime() << "[s]" << endl;
    cout << "topics: " << endl;
    for (const auto &topic_type : getTopicsWithTypes()) {
      cout << "  " << setw(50) << left << topic_type.first;
      cout << " [" << topic_type.second << "]" << endl;
    }
  }

private:
  std::string bag_in_path_;
  rosbag::Bag bag_in_;
  std::string bag_out_path_;
  rosbag::Bag bag_out_;
};