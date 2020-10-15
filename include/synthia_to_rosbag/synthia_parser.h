#ifndef SYNTHIA_TO_ROSBAG_SYNTHIA_PARSER_H_
#define SYNTHIA_TO_ROSBAG_SYNTHIA_PARSER_H_

#include <memory>
#include <map>

#include <opencv2/core/core.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "synthia_to_rosbag/synthia_common.h"

namespace synthia {

class SynthiaParser {
 public:
  // Constants for filenames for calibration files.
  static const std::string kVelToCamCalibrationFilename;
  static const std::string kCamToCamCalibrationFilename;
  static const std::string kImuToVelCalibrationFilename;
  static const std::string kVelodyneFolder;
  static const std::string kCameraFolder;
  static const std::string kPoseFolder;
  static const std::string kLeftCameraFolder;
  static const std::string kRightCameraFolder;
  static const std::string kBackFacingFolder;
  static const std::string kFrontFacingFolder;
  static const std::string kLeftFacingFolder;
  static const std::string kRightFacingFolder;
  static const std::string kTimestampFilename;
  static const std::string kDataFolder;

  static const std::string kDepthFolder;
  static const std::string kSemanticsGTFolder;
  static const std::string kRGBFolder;
  static const std::string kPosesFolder;
  static const std::string kIntrinsicsFilename;

  SynthiaParser(const std::string& dataset_path, bool rectified);

  // MAIN API: all you should need to use!
  // Loading calibration files.
  bool loadCalibration();
  void loadTimestampMaps();

  // Load specific entries (indexed by filename).
  bool getPoseAtEntry(uint64_t entry, uint64_t* timestamp,
                      synthia::Transformation* pose);
  bool getCameraPoseAtEntry(uint64_t entry, uint64_t id,
                            synthia::Transformation* pose);
  bool flipYZ(std::vector<double>* parsed_doubles);
  uint64_t getPoseTimestampAtEntry(uint64_t entry);

  bool interpolatePoseAtTimestamp(uint64_t timestamp, synthia::Transformation* pose);

  bool getGpsAtEntry() { /* TODO! */
    return false;
  }
  bool getImuAtEntry() { /* TODO! */
    return false;
  }
  std::string getCameraPath(uint64_t cam_id) {
    return cam_paths_[cam_id];
  }
  //  bool getPointcloudAtEntry(uint64_t entry, uint64_t* timestamp,
  //                            pcl::PointCloud<pcl::PointXYZI>* ptcloud);
  bool getImageAtEntry(uint64_t entry, uint64_t cam_id, uint64_t* timestamp,
                       cv::Mat* image);
  bool getDepthImageAtEntry(uint64_t entry, uint64_t cam_id,
                            uint64_t* timestamp, cv::Mat* depth_image);
  bool getLabelImageAtEntry(uint64_t entry, uint64_t cam_id,
                            uint64_t* timestamp, cv::Mat* label_image);
  bool getLabelsAtEntry(uint64_t entry, uint64_t cam_id,
                        uint64_t* timestamp, cv::Mat* label_image);

  bool getCameraCalibration(uint64_t cam_id, synthia::CameraCalibration* cam) const;

  bool convertDepthImageToDepthCloud(const sensor_msgs::Image& depth_image_msg,
                                     const sensor_msgs::Image& rgb_image_msg,
                                     const sensor_msgs::Image& labels_image_msg,
                                     const sensor_msgs::CameraInfo& cam_info,
                                     sensor_msgs::PointCloud2* ptcloud);

  synthia::Transformation T_camN_vel(int cam_number) const;
  synthia::Transformation T_camN_imu(int cam_number) const;

  // Basic accessors.
  synthia::Transformation T_cam0_vel() const;
  synthia::Transformation T_vel_imu() const;

  size_t getNumCameras() const;

 private:
  bool loadCamToCamCalibration();
  bool loadVelToCamCalibration();
  bool loadImuToVelCalibration();

  bool convertVectorToPose(const std::vector<double>& oxts, synthia::Transformation* pose);
  double latToScale(double lat) const;
  void latlonToMercator(double lat, double lon, double scale,
                        Eigen::Vector2d* mercator) const;
  bool loadTimestampsIntoVector(const std::string& filename,
                                std::vector<uint64_t>* timestamp_vec) const;

  bool parseVectorOfDoubles(const std::string& input,
                            std::vector<double>* output) const;

  //  std::string getFolderNameForCamera(int cam_number) const;
  std::string getFilenameForEntry(uint64_t entry) const;

  // Base paths.
  std::string dataset_path_;
  std::vector<std::string> cam_paths_;
  // Whether this dataset contains raw or rectified images. This determines
  // which calibration is read.
  bool rectified_;

  // Cached calibration parameters -- std::vector of camera calibrations.
  synthia::CameraCalibrationVector camera_calibrations_;

  // Transformation chain (cam-to-cam extrinsics stored above in cam calib
  // struct).
  synthia::Transformation T_cam0_vel_;
  synthia::Transformation T_vel_imu_;

  // Timestamp map from index to nanoseconds.
  // SYNTHIA has only one set of timestamps as all data is synchronized.
  std::vector<uint64_t> timestamps_ns_;
  std::vector<uint64_t> timestamps_vel_ns_;
  std::vector<uint64_t> timestamps_pose_ns_;
  // Vector of camera timestamp vectors.
  std::vector<std::vector<uint64_t> > timestamps_cam_ns_;

  // Cached pose information, to correct to odometry frame (instead of absolute
  // world coordinates).
  bool initial_pose_set_;
  synthia::Transformation T_initial_pose_;
  double mercator_scale_;
};
}

#endif  // SYNTHIA_TO_ROSBAG_SYNTHIA_PARSER_H_
