#ifndef SYNTHIA_TO_ROSBAG_SYNTHIA_ROS_CONVERSIONS_H_
#define SYNTHIA_TO_ROSBAG_SYNTHIA_ROS_CONVERSIONS_H_

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include "synthia_to_rosbag/synthia_common.h"

namespace synthia {

void calibrationToRos(uint64_t cam_id, const CameraCalibration& cam,
                      sensor_msgs::CameraInfo* cam_msg);
void stereoCalibrationToRos(const CameraCalibration& left_cam,
                            const CameraCalibration& right_cam,
                            sensor_msgs::CameraInfo* left_cam_msg,
                            sensor_msgs::CameraInfo* right_cam_msg);
void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg);
void depthImageToRos(const cv::Mat& depth_image, sensor_msgs::Image* depth_image_msg);
void poseToRos(const Transformation& transform,
               geometry_msgs::PoseStamped* pose_msg);
void posesToPath(const std::vector<geometry_msgs::PoseStamped>& poses,
                 nav_msgs::Path* path_msg);
void transformToTf(const Transformation& transform,
                   tf::Transform* tf_transform);
void transformToRos(const Transformation& transform,
                    geometry_msgs::TransformStamped* transform_msg);
void timestampToRos(uint64_t timestamp_ns, ros::Time* time);

std::string getCameraFrameId(int cam_id);

}  // namespace synthia

#endif  // SYNTHIA_TO_ROSBAG_SYNTHIA_ROS_CONVERSIONS_H_
