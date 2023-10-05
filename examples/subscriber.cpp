#include <TopicHelper.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

class SubscribingNode {

public:
  SubscribingNode() = delete;

  SubscribingNode(const std::string &image_topic,
                  const std::string &image_save_path,
                  const std::string &pose_topic, const std::string &gt_path)
      : image_topic_(image_topic), image_save_path_(image_save_path),
        pose_topic_(pose_topic), gt_path_(gt_path) {
    std::experimental::filesystem::create_directories(image_save_path_);
    pose_stamped_csv_file_.open(gt_path_);
    pose_stamped_csv_file_
        << "#timestamp,#position_x,#position_y,#position_z,"
        << "#orientation_x,#orientation_y,#orientation_z,#orientation_w"
        << std::endl;
    pose_stamped_csv_file_ << std::fixed << std::setprecision(9);

    // setup subscriber
    ros::NodeHandle nh("~");
    image_sub_ =
        nh.subscribe(image_topic_, 1000, &SubscribingNode::imageCallback, this);
    pose_sub_ =
        nh.subscribe(pose_topic_, 1000, &SubscribingNode::poseCallback, this);
  }

  ~SubscribingNode() { pose_stamped_csv_file_.close(); }

  void imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    //   ROS_INFO("[Image] I heard: [%ld]", msg->header.stamp.toNSec());

    // save image
    std::string image_file_path(image_save_path_ + "/" +
                                std::to_string(msg->header.stamp.toNSec()) +
                                ".png");
    cv::Mat image = TopicHelper::rosImage2cvImage(*msg, "mono16");
    cv::imwrite(image_file_path, image);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    //   ROS_INFO("[Pose] I heard: [%ld]", msg->header.stamp.toNSec());
    pose_stamped_csv_file_ << msg->header.stamp.toNSec() << ","
                           << msg->pose.position.x << ","
                           << msg->pose.position.y << ","
                           << msg->pose.position.z << ","
                           << msg->pose.orientation.x << ","
                           << msg->pose.orientation.y << ","
                           << msg->pose.orientation.z << ","
                           << msg->pose.orientation.w << std::endl;
  }

private:
  std::string image_topic_;
  std::string image_save_path_;
  std::string pose_topic_;
  std::string gt_path_;
  std::ofstream pose_stamped_csv_file_;
  ros::Subscriber image_sub_;
  ros::Subscriber pose_sub_;
};

int main(int argc, char **argv) {

  // initialize ROS
  ros::init(argc, argv, "rosbag_subscriber");
  ros::NodeHandle nh("~");

  // set image topic
  std::string image_topic("/adk_left/image_raw");

  // set folder to save images
  std::string save_path("/home/hj/Dataset/NSAVP/R0_RA0/images");

  // set pose topic
  std::string pose_topic("/applanix/pose_base_link");

  // set path to save groundtruth.csv
  std::string gt_path("/home/hj/Dataset/NSAVP/R0_RA0/groundtruth.csv");

  // initialize SubscribingNode
  SubscribingNode subscribing_node(image_topic, save_path, pose_topic, gt_path);

  // spin
  ros::spin();

  return 0;
}