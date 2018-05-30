// Copyright Yuto Uchimi (2018)

#include <limits>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


cv::Mat get_depth(std::string depth_png_file)
{
  cv::Mat depth_raw = cv::imread(depth_png_file, CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat depth(depth_raw.rows, depth_raw.cols, CV_32FC1);
  for (size_t j = 0; j < depth_raw.rows; ++j)
    {
      for (size_t i = 0; i < depth_raw.cols; ++i)
        {
          // FIXME
          // float tmp = static_cast<float>(depth_raw.at<unsigned short int>(j, i)) / 1000.0f;  // NOLINT
          float tmp = static_cast<float>(depth_raw.at<unsigned short int>(j, i)) / 25000.0f;  // NOLINT
          // FIXME
          // if (tmp < 0.3 || tmp > 3.0)  // nan for too small or too large depth
          if (tmp == 0.0 || tmp > 0.2)  // nan for too small or too large depth
            {
              // depth.at<float>(j, i) = std::numeric_limits<float>::quiet_NaN();
              depth.at<float>(j, i) = 0.0;
            }
          else
            {
              depth.at<float>(j, i) = tmp;
            }
        }
    }
  return depth;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr depth_to_color_pointcloud(
    int height, int width, cv::Mat depth, Eigen::Matrix3f cam_K,
    Eigen::Matrix4f cam_pose)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud->height = 1;
  cloud->width = 0;
  cloud->is_dense = false;

  for (int v = 0; v < height; v++)
    {
      for (int u = 0; u < width; u++)
        {
          float d = std::numeric_limits<float>::quiet_NaN();
          d = depth.at<float>(v, u);
          Eigen::Vector3f uv(u, v, 1);
          uv = cam_K.inverse() * uv;
          Eigen::Vector4f direction_(uv(0), uv(1), uv(2), 1);
          if (d != std::numeric_limits<float>::quiet_NaN() && d != 0.0)
            {
              direction_(0) *= d;
              direction_(1) *= d;
              direction_(2) = d;
              direction_ = cam_pose * direction_;
              Eigen::Vector3f direction(direction_(0), direction_(1), direction_(2));
              pcl::PointXYZ pt(direction(0), direction(1), direction(2));
              cloud->width++;
              cloud->points.push_back(pt);
            }
          else
            {
              continue;
            }
        }
    }
  return cloud;
}


int main(int argc, char** argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
  std::string depth_png_file = argv[1];
  cv::Mat depth = get_depth(depth_png_file);

  Eigen::Matrix3f cam_K;
  cam_K <<
    525,   0, 320,
      0, 525, 240,
      0,   0,   1;
  Eigen::Matrix4f cam_pose;
  cam_pose <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = depth_to_color_pointcloud(
      /*height=*/480, /*width=*/640, depth, cam_K, cam_pose);

  pcl::io::savePCDFileBinary("cloud.pcd", *cloud_ptr);

  return(0);
}
