# synthia_to_rosbag
Dataset tools for working with the SYNTHIA dataset raw data ( http://www.synthia-dataset.net/ ) and converting it to a ROS bag. Also allows a library for direct access to poses and images. This is an adaptation of the kitti_to_rosbag tool by Helen Oleynikova ( https://github.com/ethz-asl/kitti_to_rosbag ).

## Rosbag converter usage example
```
rosrun synthia_to_rosbag synthia_rosbag_converter dataset_path output_path
```
(No trailing slashes). Note: the library currently assumes the original SYNTHIA structure of the dataset subfolders.

```
rosrun synthia_to_rosbag synthia_rosbag_converter /media/johnny/082e0614-ce4c-45cc-abc5-dbde7ae882bb/SYNTHIA/Video_sequences/SYNTHIA-SEQS-04-SUMMER /media/johnny/082e0614-ce4c-45cc-abc5-dbde7ae882bb/SYNTHIA/Video_sequences/SYNTHIA-SEQS-04-SUMMER/outbag.bag
```

## Library API Example
```C++
#include <opencv2/highgui/highgui.hpp>

#include "synthia_to_rosbag/synthia_parser.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  
  const std::string dataset_path =
      "/media/johnny/082e0614-ce4c-45cc-abc5-dbde7ae882bb/SYNTHIA/Video_sequences/SYNTHIA-SEQS-04-SUMMER";

  synthia::SynthiaParser parser(dataset_path, true);

  parser.loadCalibration();
  parser.loadTimestampMaps();

  uint64_t timestamp;
  synthia::Transformation pose;
  parser.getPoseAtEntry(0, &timestamp, &pose);

  std::cout << "Timestamp: " << timestamp << " Pose: " << pose << std::endl;

  pcl::PointCloud<pcl::PointXYZI> ptcloud;
  parser.getPointcloudAtEntry(0, &timestamp, &ptcloud);

  std::cout << "Timestamp: " << timestamp << " Num points: " << ptcloud.size()
            << std::endl;

  cv::Mat image;
  parser.getImageAtEntry(0, 3, &timestamp, &image);

  cv::imshow("Display window", image);
  cv::waitKey(0);

  return 0;
}
  
```
