// CopyRight [2018] <Yuto Uchimi>

#include <Eigen/Core>
#include <iostream>
#include <time.h>

#include <pcl/console/print.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/gpu/kinfu_large_scale/cyclical_buffer.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
// FIXME
// #include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
// #include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>

#include "standalone_marching_cubes.hpp"
#include <internal.h>

// FIXME
#include <tables.h>


// FIXME
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted(
    new pcl::PointCloud<pcl::PointXYZ>());

// pcl::gpu::kinfuLS::MarchingCubes::MarchingCubes()
// {
//   edgeTable_.upload((const int*)edgeTable, 256);
//   numVertsTable_.upload(numVertsTable, 256);
//   triTable_.upload(&triTable[0][0], 256 * 16);
// }


pcl::gpu::DeviceArray<pcl::gpu::kinfuLS::MarchingCubes::PointType>
pcl::gpu::kinfuLS::MarchingCubes::run(
    const TsdfVolume& tsdf, DeviceArray<PointType>& triangles_buffer)
{
  if (triangles_buffer.empty())
    triangles_buffer.create(DEFAULT_TRIANGLES_BUFFER_SIZE);
  occupied_voxels_buffer_.create(3, triangles_buffer.size() / 3);

  pcl::device::kinfuLS::bindTextures(edgeTable_, triTable_, numVertsTable_);

  // FIXME
  pcl::gpu::DeviceArray<pcl::PointXYZ> cloud_buffer_device;
  pcl::gpu::DeviceArray<pcl::PointXYZ> extracted = tsdf.fetchCloud(
      cloud_buffer_device);
  extracted.download(cloud_extracted->points);
  cloud_extracted->width = static_cast<int>(cloud_extracted->points.size());
  cloud_extracted->height = 1;

  // INFO
  // std::cout << "tsdf.data(): " << tsdf.data() << std::endl;
  // std::cout << "tsdf.getSize() [m]: " << std::endl << tsdf.getSize() <<
  //   std::endl;
  // std::cout << "tsdf.getResolution(): " << std::endl << tsdf.getResolution() <<
  //   std::endl;
  // std::cout << "tsdf.getVoxelSize() [m]: " << std::endl <<
  //   tsdf.getVoxelSize() << std::endl;
  // std::cout << "tsdf.getTsdfTruncDist() [m]: " << tsdf.getTsdfTruncDist() <<
  //   std::endl;
  // std::cout << "triangles_buffer: " << triangles_buffer << std::endl;
  // std::cout << "triangles_buffer.size(): " << triangles_buffer.size() <<
  //   std::endl;
  // std::cout << "occupied_voxels_buffer_: " << occupied_voxels_buffer_ <<
  //   std::endl;
  // std::cout << "occupied_voxels_buffer_.ptr(0): " <<
  //   occupied_voxels_buffer_.ptr(0) << std::endl;
  // std::cout << "occupied_voxels_buffer_.ptr(1): " <<
  //   occupied_voxels_buffer_.ptr(1) << std::endl;
  // std::cout << "occupied_voxels_buffer_.rows(): " <<
  //   occupied_voxels_buffer_.rows() << std::endl;
  // std::cout << "occupied_voxels_buffer_.cols(): " <<
  //   occupied_voxels_buffer_.cols() << std::endl;
  // FAIL: This returns 0
  int active_voxels = pcl::device::kinfuLS::getOccupiedVoxels(
      tsdf.data(), occupied_voxels_buffer_);
  // INFO
  std::cout << "active_voxels: " << active_voxels << std::endl;
  if (!active_voxels)
    {
      // INFO
      std::cout << "[ERROR] No active_voxels." << std::endl;
      pcl::device::kinfuLS::unbindTextures();
      return DeviceArray<PointType>();
    }

  DeviceArray2D<int> occupied_voxels(
      3, active_voxels, occupied_voxels_buffer_.ptr(),
      occupied_voxels_buffer_.step());

  int total_vertexes = pcl::device::kinfuLS::computeOffsetsAndTotalVertexes(
      occupied_voxels);

  float3 volume_size = pcl::device::kinfuLS::device_cast<const float3>(
      tsdf.getSize());
  pcl::device::kinfuLS::generateTriangles(
      tsdf.data(), occupied_voxels, volume_size,
      (DeviceArray<pcl::device::kinfuLS::PointType>&)triangles_buffer);

  pcl::device::kinfuLS::unbindTextures();
  return DeviceArray<PointType>(triangles_buffer.ptr(), total_vertexes);
}


int main(int argc, char** argv)
{
  std::cout << std::endl;

  if (argc != 2) {
    std::cerr << "[ERROR] Usage: 'marching_cubes_gpu OBJ_FILE'" << std::endl <<
      std::endl;
    exit(1);
  }

  pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

  // Load OBJ file
  std::cout << "Start loading: " << argv[1] << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadOBJFile(argv[1], *cloud) == -1)
  {
    std::cerr << "[ERROR] Couldn't load OBJ file. Specified file name: " <<
      argv[1] << std::endl << std::endl;
    exit(1);
  }
  std::cout << "Finished loading: " << argv[1] << std::endl;

  // Create PointCloud with intensity
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(
      new pcl::PointCloud<pcl::PointXYZI>());
  cloud_xyzi->points.resize(cloud->size());
  pcl::copyPointCloud(*cloud, *cloud_xyzi);
  for (size_t i = 0; i < cloud->points.size(); i++) {
    cloud_xyzi->points[i].x += 0.2;
    cloud_xyzi->points[i].y += 0.2;
    cloud_xyzi->points[i].z += 0.8;
    cloud_xyzi->points[i].intensity = 1.0f;
  }

  // Save as .pcd
  pcl::io::savePCDFileASCII("model.pcd", *cloud_xyzi);

  // Apply marching cubes algorithm
  int device = 0;
  pcl::gpu::setDevice(device);
  pcl::gpu::printShortCudaDeviceInfo(device);
  pcl::gpu::kinfuLS::StandaloneMarchingCubes< pcl::PointCloud<pcl::PointXYZI> > smc(
      /*voxel_x=*/512, /*voxel_y=*/512, /*voxel_z=*/512, /*volume_size=*/3.0f);
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
  std::cout << "Start marching cubes reconstruction..." << std::endl;
  clock_t start = clock();
  std::cout << "triangles: " << triangles << std::endl;
  triangles = smc.getMeshFromTSDFCloud(*cloud_xyzi);
  std::cout << "triangles: " << triangles << std::endl;
  clock_t end = clock();
  std::cout << "Reconstruction finished!" << std::endl;

  std::cout << "Elapsed time on GPU: " <<
    static_cast<double>(end - start) / CLOCKS_PER_SEC << " [sec]" <<
    std::endl << std::endl;
  std::cout << "cloud_xyzi->points.size(): " << cloud_xyzi->points.size() << std::endl;
  std::cout << "cloud_extracted->points.size(): " << cloud_extracted->points.size() << std::endl;

  // Visualization
  pcl::visualization::PCLVisualizer viewer("viewer");
  viewer.setBackgroundColor(0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZI>(cloud_xyzi, "cloud_xyzi", 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_extracted, "cloud_extracted", 0);
  viewer.addPolygonMesh(*triangles, "triangles", 0);
  viewer.addCoordinateSystem(0.1);
  viewer.initCameraParameters();
  viewer.setCameraPosition(
      /*pos=*/0.4, 0.0, 0.0, /*view=*/0.2, 0.2, 0.8, /*up=*/0.0, 1.0, 0.0,
      /*viewport=*/0);
  while (!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

  return(0);
}
