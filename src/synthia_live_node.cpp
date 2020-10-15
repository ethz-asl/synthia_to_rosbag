#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>

#include "../include/synthia_to_rosbag/synthia_parser.h"
#include "../include/synthia_to_rosbag/synthia_ros_conversions.h"

namespace synthia {

class SynthiaLiveNode {
 public:
  SynthiaLiveNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                  const std::string& dataset_path);

  // Creates a timer to automatically publish entries in 'realtime' versus
  // the original data,
  void startPublishing(double rate_hz);

  bool publishEntry(uint64_t entry);

  void publishTf(uint64_t timestamp_ns, const synthia::Transformation& imu_pose);

  void publishClock(uint64_t timestamp_ns);

 private:
  void timerCallback(const ros::WallTimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers for the topics.
  ros::Publisher clock_pub_;
  //  ros::Publisher pointcloud_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher transform_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  image_transport::ImageTransport image_transport_;
  std::vector<image_transport::CameraPublisher> image_pubs_;

  // Decides the rate of publishing (TF is published at this rate).
  // Call startPublishing() to turn this on.
  ros::WallTimer publish_timer_;

  synthia::SynthiaParser parser_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string velodyne_frame_id_;

  uint64_t current_entry_;
  uint64_t publish_dt_ns_;
  uint64_t current_timestamp_ns_;
};

SynthiaLiveNode::SynthiaLiveNode(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private,
                                 const std::string& dataset_path)
: nh_(nh),
  nh_private_(nh_private),
  image_transport_(nh_),
  parser_(dataset_path, true),
  world_frame_id_("world"),
  imu_frame_id_("imu"),
  cam_frame_id_prefix_("cam"),
  velodyne_frame_id_("velodyne"),
  current_entry_(0),
  publish_dt_ns_(0),
  current_timestamp_ns_(0) {
  // Load all the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  parser_.loadTimestampMaps();

  // Advertise all the publishing topics for ROS live streaming.
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1, false);
  pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("pose_imu", 10, false);
  transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
      "transform_imu", 10, false);

  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    image_pubs_.push_back(
        image_transport_.advertiseCamera(synthia::getCameraFrameId(cam_id), 1));
  }
}

void SynthiaLiveNode::startPublishing(double rate_hz) {
  double publish_dt_sec = 1.0 / rate_hz;
  publish_dt_ns_ = static_cast<uint64_t>(publish_dt_sec * 1e9);
  publish_timer_ = nh_.createWallTimer(ros::WallDuration(publish_dt_sec),
                                       &SynthiaLiveNode::timerCallback, this);
}

void SynthiaLiveNode::timerCallback(const ros::WallTimerEvent& event) {
  synthia::Transformation tf_interpolated;

  std::cout << "Current entry: " << current_entry_ << std::endl;

  if (current_entry_ == 0) {
    // This is the first time this is running! Initialize the current timestamp
    // and publish this entry.
    if (!publishEntry(current_entry_)) {
      publish_timer_.stop();
    }
    current_timestamp_ns_ = parser_.getPoseTimestampAtEntry(current_entry_);
    publishClock(current_timestamp_ns_);
    if (parser_.interpolatePoseAtTimestamp(current_timestamp_ns_,
                                           &tf_interpolated)) {
      publishTf(current_timestamp_ns_, tf_interpolated);
    }
    current_entry_++;
    return;
  }

  current_timestamp_ns_ += publish_dt_ns_;
  publishClock(current_timestamp_ns_);
  if (parser_.interpolatePoseAtTimestamp(current_timestamp_ns_,
                                         &tf_interpolated)) {
    publishTf(current_timestamp_ns_, tf_interpolated);
  } else {
    std::cout << "Failed to interpolate!\n";
  }

  std::cout << "Current entry's timestamp: "
      << parser_.getPoseTimestampAtEntry(current_entry_) << std::endl;
  if (parser_.getPoseTimestampAtEntry(current_entry_) <=
      current_timestamp_ns_) {
    if (!publishEntry(current_entry_)) {
      publish_timer_.stop();
      return;
    }
    current_entry_++;
  }
}

void SynthiaLiveNode::publishClock(uint64_t timestamp_ns) {
  ros::Time timestamp_ros;
  synthia::timestampToRos(timestamp_ns, &timestamp_ros);
  rosgraph_msgs::Clock clock_time;
  clock_time.clock = timestamp_ros;
  clock_pub_.publish(clock_time);
}

bool SynthiaLiveNode::publishEntry(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;
  rosgraph_msgs::Clock clock_time;

  // Publish poses + TF transforms + clock.
  synthia::Transformation pose;
  if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TransformStamped transform_msg;

    synthia::timestampToRos(timestamp_ns, &timestamp_ros);
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.header.stamp = timestamp_ros;
    transform_msg.header.frame_id = world_frame_id_;
    transform_msg.header.stamp = timestamp_ros;

    synthia::poseToRos(pose, &pose_msg);
    synthia::transformToRos(pose, &transform_msg);

    pose_pub_.publish(pose_msg);
    transform_pub_.publish(transform_msg);

  } else {
    return false;
  }

  // Publish images.
  cv::Mat image;
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image)) {
      sensor_msgs::Image image_msg;
      synthia::imageToRos(image, &image_msg);

      // Get the calibration info for this camera.
      synthia::CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      synthia::calibrationToRos(cam_id, cam_calib, &cam_info);

      synthia::timestampToRos(timestamp_ns, &timestamp_ros);

      image_msg.header.stamp = timestamp_ros;
      image_msg.header.frame_id = synthia::getCameraFrameId(cam_id);
      cam_info.header = image_msg.header;

      image_pubs_[cam_id].publish(image_msg, cam_info, timestamp_ros);
    }
  }

  return true;
}

void SynthiaLiveNode::publishTf(uint64_t timestamp_ns,
                                const synthia::Transformation& imu_pose) {
  ros::Time timestamp_ros;
  synthia::timestampToRos(timestamp_ns, &timestamp_ros);
  synthia::Transformation T_imu_world = imu_pose;
  synthia::Transformation T_vel_imu = parser_.T_vel_imu();
  synthia::Transformation T_cam_imu;

  tf::Transform tf_imu_world, tf_cam_imu, tf_vel_imu;

  synthia::transformToTf(T_imu_world, &tf_imu_world);
  synthia::transformToTf(T_vel_imu.inverse(), &tf_vel_imu);

  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_imu_world, timestamp_ros, world_frame_id_, imu_frame_id_));
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_vel_imu, timestamp_ros, imu_frame_id_, velodyne_frame_id_));

  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    T_cam_imu = parser_.T_camN_imu(cam_id);
    synthia::transformToTf(T_cam_imu.inverse(), &tf_cam_imu);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        tf_cam_imu, timestamp_ros, imu_frame_id_, synthia::getCameraFrameId(cam_id)));
  }
}

}  // namespace synthia

int main(int argc, char** argv) {
  ros::init(argc, argv, "synthia_live");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  const std::string dataset_path =
      "/media/johnny/082e0614-ce4c-45cc-abc5-dbde7ae882bb/SYNTHIA/Video_sequences/SYNTHIA-SEQS-04-SUMMER";

  synthia::SynthiaLiveNode node(nh, nh_private, dataset_path);

  node.startPublishing(50.0);

  ros::spin();

  return 0;
}
