#include <cv_bridge/cv_bridge.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include "synthia_to_rosbag/synthia_ros_conversions.h"


namespace synthia {

std::string getCameraFrameId(int cam_id) {
  char buffer[20];
  sprintf(buffer, "cam%02d", cam_id);
  return std::string(buffer);
}

void calibrationToRos(uint64_t cam_id, const CameraCalibration& cam,
                      sensor_msgs::CameraInfo* cam_msg) {
  cam_msg->header.frame_id = getCameraFrameId(cam_id);

  cam_msg->width = cam.image_size.x();
  cam_msg->height = cam.image_size.y();

  cam_msg->distortion_model = "plumb_bob";

  // D is otherwise empty by default, hopefully this is fine.
  if (cam.distorted) {
    const size_t kNumDistortionParams = 5;
    cam_msg->D.resize(kNumDistortionParams);
    for (size_t i = 0; i < kNumDistortionParams; ++i) {
      cam_msg->D[i] = cam.D(i);
    }
  }

  // Copy over intrinsics.
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      cam_msg->K[3 * i + j] = cam.K(i, j);
    }
  }

  // Rectification/projection matrices.
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      cam_msg->R[3 * i + j] = cam.rect_mat(i, j);
    }
  }

  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      cam_msg->P[4 * i + j] = cam.projection_mat(i, j);
    }
  }
}

void stereoCalibrationToRos(uint64_t left_cam_id, uint64_t right_cam_id,
                            const CameraCalibration& left_cam,
                            const CameraCalibration& right_cam,
                            sensor_msgs::CameraInfo* left_cam_msg,
                            sensor_msgs::CameraInfo* right_cam_msg) {
  // Fill in the basics for each camera.
  calibrationToRos(left_cam_id, left_cam, left_cam_msg);
  calibrationToRos(right_cam_id, right_cam, right_cam_msg);
}

void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg) {
  cv_bridge::CvImage image_cv_bridge;
  image_cv_bridge.image = image;

  if (image.type() == CV_8U) {
    image_cv_bridge.encoding = "mono8";
  } else if (image.type() == CV_8UC3) {
    image_cv_bridge.encoding = "bgr8";
  }
  image_cv_bridge.toImageMsg(*image_msg);
}

void depthImageToRos(const cv::Mat& depth_image, sensor_msgs::Image* depth_image_msg) {
  cv_bridge::CvImage image_cv_bridge;
  image_cv_bridge.image = depth_image;

  if (depth_image.type() == CV_8UC1) {
    image_cv_bridge.encoding = "mono8";
  } else if (depth_image.type() == CV_16UC1) {
    image_cv_bridge.encoding = "mono16";
  } else if (depth_image.type() == CV_8UC3) {
    image_cv_bridge.encoding = "bgr8";
  } else if (depth_image.type() == CV_8UC4) {
    image_cv_bridge.encoding = "bgra8";
  }
  image_cv_bridge.toImageMsg(*depth_image_msg);
}

void poseToRos(const Transformation& transform,
               geometry_msgs::PoseStamped* pose_msg) {
  tf::poseKindrToMsg(transform, &pose_msg->pose);
}

void posesToPath(const std::vector<geometry_msgs::PoseStamped>& poses,
                 nav_msgs::Path* path_msg) {
  if(poses.size() > 0) {
    path_msg->poses.resize(poses.size());
    for (size_t i = 0u; i < poses.size(); ++i) {
      path_msg->poses[i] = poses[i];
    }
  }
}

void transformToTf(const Transformation& transform,
                   tf::Transform* tf_transform) {
  tf::transformKindrToTF(transform, tf_transform);
}

void transformToRos(const Transformation& transform,
                    geometry_msgs::TransformStamped* transform_msg) {
  tf::transformKindrToMsg(transform, &transform_msg->transform);
}

void timestampToRos(uint64_t timestamp_ns, ros::Time* time) {
  time->fromNSec(timestamp_ns);
}

}  // namespace synthia
