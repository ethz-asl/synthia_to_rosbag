#include "synthia_to_rosbag/synthia_parser.h"
#include "synthia_to_rosbag/depth_traits.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include "ros/ros.h"

#include <eigen-checks/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <opencv2/highgui/highgui.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>

namespace synthia {

const std::string SynthiaParser::kDepthFolder = "Depth";
const std::string SynthiaParser::kSemanticsGTFolder = "GT";
const std::string SynthiaParser::kRGBFolder = "RGB";
const std::string SynthiaParser::kPosesFolder = "CameraParams";
const std::string SynthiaParser::kLeftCameraFolder = "Stereo_Left";
const std::string SynthiaParser::kRightCameraFolder = "Stereo_Right";
const std::string SynthiaParser::kBackFacingFolder = "Omni_B";
const std::string SynthiaParser::kFrontFacingFolder = "Omni_F";
const std::string SynthiaParser::kLeftFacingFolder = "Omni_L";
const std::string SynthiaParser::kRightFacingFolder = "Omni_R";
const std::string SynthiaParser::kIntrinsicsFilename = "intrinsics.txt";

SynthiaParser::SynthiaParser(const std::string& dataset_path, bool rectified)
    : dataset_path_(dataset_path),
      rectified_(rectified),
      initial_pose_set_(false) {
  cam_paths_.push_back(kLeftCameraFolder + "/" + kFrontFacingFolder);
  cam_paths_.push_back(kLeftCameraFolder + "/" + kRightFacingFolder);
  cam_paths_.push_back(kLeftCameraFolder + "/" + kBackFacingFolder);
  cam_paths_.push_back(kLeftCameraFolder + "/" + kLeftFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kFrontFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kRightFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kBackFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kLeftFacingFolder);
}

bool SynthiaParser::loadCalibration() {

  camera_calibrations_.resize(cam_paths_.size());
  size_t cam_id = 0u;
  for (auto &&calibration : camera_calibrations_) {
    // Intrinsics.
    calibration.image_size << 1280, 760;
    calibration.rect_mat = Eigen::Matrix3d::Identity();
    calibration.projection_mat = Eigen::Matrix<double, 3, 4>::Identity();
    calibration.K = Eigen::Matrix3d::Identity();
    calibration.K(0, 0) = 532.740352;
    calibration.K(1, 1) = 532.740352;
    calibration.K(0, 2) = 640;
    calibration.K(1, 2) = 380;

    calibration.projection_mat(0, 0) = 532.740352;
    calibration.projection_mat(1, 1) = 532.740352;
    calibration.projection_mat(0, 2) = 640;
    calibration.projection_mat(1, 2) = 380;

    calibration.D = Eigen::Matrix<double, 1, 5>::Zero();

    // Extrinsics are loaded from file.
  }
  return true;
}

bool SynthiaParser::parseVectorOfDoubles(const std::string& input,
                                         std::vector<double>* output) const {
  output->clear();
  // Parse the line as a stringstream for space-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ' ');
    if (element.empty()) {
      continue;
    }
    try {
      output->emplace_back(std::stod(element));
    } catch (const std::exception &exception) {
      std::cout << "Could not parse number in import file.\n";
      return false;
    }
  }
  return true;
}

void SynthiaParser::loadTimestampMaps() {

  std::string file_path = dataset_path_ + "/CameraParams/Stereo_Right/Omni_F";
  size_t num_files = 0u;
  for (boost::filesystem::directory_iterator it(file_path);
       it != boost::filesystem::directory_iterator(); ++it) {
    timestamps_ns_.push_back(1 + num_files * 2e8);
    num_files++;
  }
}

bool SynthiaParser::getCameraCalibration(uint64_t cam_id,
                                         synthia::CameraCalibration* cam)
const {
  if (cam_id >= camera_calibrations_.size()) {
    return false;
  }
  *cam = camera_calibrations_[cam_id];
  return true;
}

bool SynthiaParser::convertDepthImageToDepthCloud(
    const sensor_msgs::Image& depth_image_msg,
    const sensor_msgs::Image& rgb_image_msg,
    const sensor_msgs::Image& labels_image_msg,
    const sensor_msgs::CameraInfo& cam_info,
    sensor_msgs::PointCloud2 *ptcloud) {
  // Function adapted from https://github.com/ros-perception/image_pipeline/tree/indigo/depth_image_proc.
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(cam_info);
  // Use correct principal point from calibration.
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for
  // computing (X,Y).
  double unit_scaling = DepthTraits<uint16_t>::toMeters(uint16_t(1));
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  // Reset fields and reserve 7 fields = XYZRGBL
  ptcloud->fields.clear();
  ptcloud->fields.reserve(7);

  // Set fields
  int offset = 0;
  offset =
      addPointField(*ptcloud, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset =
      addPointField(*ptcloud, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset =
      addPointField(*ptcloud, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(*ptcloud,
                         "rgb",
                         1,
                         sensor_msgs::PointField::FLOAT32,
                         offset);
  offset = addPointField(*ptcloud,
                         "label",
                         1,
                         sensor_msgs::PointField::UINT8,
                         offset);

  // Resize Pointcloud properly.
  ptcloud->point_step = offset;
  ptcloud->row_step = ptcloud->width * ptcloud->point_step;
  ptcloud->data.resize(ptcloud->height * ptcloud->row_step);

  sensor_msgs::PointCloud2Modifier pcd_modifier(*ptcloud);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*ptcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*ptcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*ptcloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*ptcloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*ptcloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*ptcloud, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(*ptcloud, "label");

  const uint16_t
      *depth_row = reinterpret_cast<const uint16_t *>(&depth_image_msg.data[0]);
  const uint8_t *rgb = &rgb_image_msg.data[0];
  const uint8_t *labels = &labels_image_msg.data[0];
  int color_step = 3;
  // Labels come as 16bit sRGB therefore we need to shift that 1-byte pointer
  // double.
  int labels_step = 6;
  uint16_t dr = depth_image_msg.data[0];
  int row_step = depth_image_msg.step / sizeof(uint16_t);

  for (int v = 0; v < (int) ptcloud->height; ++v, depth_row += row_step) {
    for (int u = 0; u < (int) ptcloud->width; ++u, ++iter_x, ++iter_y,
        ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++iter_label, rgb += color_step,
                                                              labels +=
                                                                  labels_step) {
      uint16_t depth = uint16_t(depth_row[u]);
      if (DepthTraits<uint16_t>::toMeters(depth) < 200) {
        // Fill in XYZ.
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = DepthTraits<uint16_t>::toMeters(depth);
        // Images read by opencv are bgr -> swap channels for rgb pointcloud.
        *iter_r = rgb[2];
        *iter_g = rgb[1];
        *iter_b = rgb[0];
        // Channel: 0: empty; 1: class labels; 2: instance labels
        // Recall we use a 8bit pointer in a 16bit array therefore: 0=0 ; 1=3
        // ; 2=4.
        *iter_label = (uint8_t) labels[4];
      }
    }
  }

  // Perform random downsampling of point cloud.
  pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*ptcloud, *pcl_pc2);
  pcl::RandomSample<pcl::PCLPointCloud2> filter;
  filter.setInputCloud(pcl_pc2);
  filter.setSample(30000);
  filter.filter(*pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(*pcl_pc2, *source_pointcloud);

  // Pass through filtering.
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setFilterFieldName("x");
  pass.setNegative(true);
  pass.setFilterLimits(-0.001f, 0.001f);
  pass.setInputCloud(source_pointcloud);
  pass.filter(*source_pointcloud);

  // Relabel flipped color channels.
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it =
      source_pointcloud->begin(); it != source_pointcloud->end(); it++) {
    u_int8_t temp = it->b;
    it->b = it->r;
    it->r = temp;
  }

  // Rotate point-cloud into correct orientation.
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(1, 1) = -1;
  transform(1, 2) = 0;
  transform(2, 1) = 0;
  transform(2, 2) = -1;
  pcl::transformPointCloud(*source_pointcloud, *source_pointcloud, transform);
  pcl::toROSMsg(*source_pointcloud, *ptcloud);
  ptcloud->height = 1;
  ptcloud->width = source_pointcloud->size();

  return true;
}

bool SynthiaParser::getPoseAtEntry(uint64_t entry, uint64_t* timestamp,
                                   synthia::Transformation* pose) {
  // Read in both camera poses and spawn one in between.
  std::string filename_cam0 = dataset_path_ + "/" + kPosesFolder +
      "/Stereo_Left/Omni_F/" + getFilenameForEntry(entry) + ".txt";
  std::string filename_cam1 = dataset_path_ + "/" + kPosesFolder +
      "/Stereo_Right/Omni_F/" + getFilenameForEntry(entry) + ".txt";

  std::ifstream import_file_cam0(filename_cam0, std::ios::in);
  std::ifstream import_file_cam1(filename_cam1, std::ios::in);
  if (!import_file_cam0 || !import_file_cam1) {
    return false;
  }
  if (timestamps_ns_.size() <= entry) {
    return false;
  }
  *timestamp = timestamps_ns_[entry];

  std::vector<double> parsed_doubles_0, parsed_doubles_1;
  std::string line_0, line_1;
  std::getline(import_file_cam0, line_0);
  std::getline(import_file_cam1, line_1);
  if (parseVectorOfDoubles(line_0, &parsed_doubles_0) &&
      parseVectorOfDoubles(line_1, &parsed_doubles_1)) {
    // Flip z,y.
    if (!flipYZ(&parsed_doubles_0) || !flipYZ(&parsed_doubles_1)) {
      return false;
    }
    // Make position between two cameras.
    parsed_doubles_0[12] = (parsed_doubles_0[12] + parsed_doubles_1[12]) / 2;
    parsed_doubles_0[13] = (parsed_doubles_0[13] + parsed_doubles_1[13]) / 2;
    parsed_doubles_0[14] = (parsed_doubles_0[14] + parsed_doubles_1[14]) / 2;

    if (convertVectorToPose(parsed_doubles_0, pose)) {
      return true;
    }
  }
  return false;
}

bool SynthiaParser::getCameraPoseAtEntry(uint64_t entry, uint64_t id,
                                         synthia::Transformation* pose) {
  // Read in both camera poses and spawn one in between.
  std::string filename_cam = dataset_path_ + "/" + kPosesFolder +
      "/" + cam_paths_[id] + "/" + getFilenameForEntry(entry) + ".txt";

  std::ifstream import_file_cam(filename_cam, std::ios::in);
  if (!import_file_cam) {
    return false;
  }
  std::vector<double> parsed_doubles;
  std::string line;
  std::getline(import_file_cam, line);
  if (parseVectorOfDoubles(line, &parsed_doubles)) {
    // Flip z,y.
    if (!flipYZ(&parsed_doubles)) {
      return false;
    }
    if (convertVectorToPose(parsed_doubles, pose)) {
      return true;
    }
  }
  return false;
}

bool SynthiaParser::flipYZ(std::vector<double>* parsed_doubles) {
  if (parsed_doubles->size() == 16) {
    std::iter_swap(parsed_doubles->begin() + 1, parsed_doubles->begin() + 2);
    std::iter_swap(parsed_doubles->begin() + 5, parsed_doubles->begin() + 6);
    std::iter_swap(parsed_doubles->begin() + 9, parsed_doubles->begin() + 10);
    std::iter_swap(parsed_doubles->begin() + 13, parsed_doubles->begin() + 14);
    return true;
  } else {
    return false;
  }
}

uint64_t SynthiaParser::getPoseTimestampAtEntry(uint64_t entry) {
  if (timestamps_ns_.size() <= entry) {
    return 0;
  }
  return timestamps_ns_[entry];
}

bool SynthiaParser::getImageAtEntry(uint64_t entry, uint64_t cam_id,
                                    uint64_t* timestamp, cv::Mat* image) {
  // Get the timestamp for this first.
  std::string filename = dataset_path_ + "/RGB/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);

  if (!image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::getDepthImageAtEntry(uint64_t entry,
                                         uint64_t cam_id,
                                         uint64_t* timestamp,
                                         cv::Mat* depth_image) {
  // Get the timestamp for this first.

  std::string filename = dataset_path_ + "/Depth/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *depth_image = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);

  if (!depth_image->data) {
    std::cout << "Could not load depth image data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::getLabelImageAtEntry(uint64_t entry,
                                         uint64_t cam_id,
                                         uint64_t* timestamp,
                                         cv::Mat* label_image) {
  // Get the timestamp for this first.
  *timestamp = timestamps_ns_[entry];

  std::string
      filename = dataset_path_ + "/GT/COLOR/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *label_image =
      cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

  if (!label_image->data) {
    std::cout << "Could not load labels image data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::getLabelsAtEntry(uint64_t entry,
                                     uint64_t cam_id,
                                     uint64_t* timestamp,
                                     cv::Mat* label_image) {
  // Get the timestamp for this first.
  std::cout << "entry: " << entry << std::endl;

  std::string
      filename = dataset_path_ + "/GT/LABELS/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *label_image =
      cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

  if (!label_image->data) {
    std::cout << "Could not load label data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::convertVectorToPose(const std::vector<double>& pose_16,
                                        synthia::Transformation* pose) {
  if (pose_16.size() < 16) {
    return false;
  }
  std::vector<double> pose_temp = pose_16;
  typedef Eigen::Matrix<double, 4, 4, Eigen::ColMajor> TransformationMatrix;
  Eigen::Map<TransformationMatrix> m1(pose_temp.data(), 4, 4);
  TransformationMatrix mat = m1;
  Eigen::Quaternion<double> quat(mat.topLeftCorner<3,3>());
  mat.topLeftCorner<3,3>() = quat.normalized().toRotationMatrix();
  synthia::Transformation t_temp(mat);
  *pose = t_temp;

  return true;
}

// From the MATLAB raw data dev kit.
double SynthiaParser::latToScale(double lat) const {
  return cos(lat * M_PI / 180.0);
}

// From the MATLAB raw data dev kit.
void SynthiaParser::latlonToMercator(double lat, double lon, double scale,
                                     Eigen::Vector2d* mercator) const {
  double er = 6378137;
  mercator->x() = scale * lon * M_PI * er / 180.0;
  mercator->y() = scale * er * log(tan((90.0 + lat) * M_PI / 360.0));
}

std::string SynthiaParser::getFilenameForEntry(uint64_t entry) const {
  char buffer[20];
  sprintf(buffer, "%06d", entry);
  return std::string(buffer);
}

synthia::Transformation SynthiaParser::T_camN_vel(int cam_number) const {
  return camera_calibrations_[cam_number].T_cam0_cam.inverse() * T_cam0_vel_;
}

synthia::Transformation SynthiaParser::T_camN_imu(int cam_number) const {
  return T_camN_vel(cam_number) * T_vel_imu_;
}

synthia::Transformation SynthiaParser::T_cam0_vel() const { return T_cam0_vel_;}

synthia::Transformation SynthiaParser::T_vel_imu() const { return T_vel_imu_;}

bool SynthiaParser::interpolatePoseAtTimestamp(uint64_t timestamp,
                                               synthia::Transformation* pose) {
  // Look up the closest 2 timestamps to this.
  size_t left_index = timestamps_ns_.size();
  for (size_t i = 0; i < timestamps_ns_.size(); ++i) {
    if (timestamps_ns_[i] > timestamp) {
      if (i == 0) {
        // Then we can't interpolate the pose since we're outside the range.
        return false;
      }
      left_index = i - 1;
      break;
    }
  }
  if (left_index >= timestamps_ns_.size()) {
    return false;
  }
  // Figure out what 't' should be, where t = 0 means 100% left boundary,
  // and t = 1 means 100% right boundary.
  double t = (timestamp - timestamps_ns_[left_index]) /
      static_cast<double>(timestamps_ns_[left_index + 1] -
          timestamps_ns_[left_index]);

  // Load the two transformations.
  uint64_t timestamp_left, timestamp_right;
  synthia::Transformation transform_left, transform_right;
  if (!getPoseAtEntry(left_index, &timestamp_left, &transform_left) ||
      !getPoseAtEntry(left_index + 1, &timestamp_right, &transform_right)) {
    // For some reason couldn't load the poses.
    return false;
  }

  // Interpolate between them.
  *pose =
      synthia::interpolateTransformations(transform_left, transform_right, t);
  return true;
}

size_t SynthiaParser::getNumCameras() const {
  return camera_calibrations_.size();
}

}  // namespace synthia

