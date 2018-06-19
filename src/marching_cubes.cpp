// CopyRight [2018] <Yuto Uchimi>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <time.h>

#include <pcl/console/print.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  std::cout << std::endl;

  std::string file, output;
  float iso_level;
  int grid_res, decimate_thre;

  namespace po = boost::program_options;
  po::options_description desc("option");
  desc.add_options()
    ("file,f", po::value<std::string>(&file)->required(),
     "The loaded file name")
    ("output,o", po::value<std::string>(&output)->required(),
     "The output OBJ file name")
    ("iso_level,i", po::value<float>(&iso_level)->default_value(0.00f),
     "The ISO level")
    ("grid_res,g", po::value<int>(&grid_res)->default_value(100),
     "The grid resolution")
    ("decimate_thre,d", po::value<int>(&decimate_thre)->default_value(5000),
     "The decimate threshold");

  po::variables_map vm;
  try
    {
      po::store(po::parse_command_line(argc, argv, desc), vm);
    }
  catch(const po::error_with_option_name& e)
    {
      std::cout << "[ERROR] " << e.what() << std::endl;
      exit(1);
    }
  po::notify(vm);

  pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

  // Load file
  std::cout << "Start loading: " << file << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
  if (pcl::io::loadOBJFile(file, *cloud) == -1 ||
      pcl::io::loadOBJFile(file, *mesh) == -1)
  {
    if (pcl::io::loadPCDFile(file, *cloud) == -1)
      {
        std::cerr <<
          "[ERROR] Couldn't load OBJ or PCD file. Specified file name: " <<
          file << std::endl << std::endl;
        exit(1);
      }
  }
  std::cout << "Finished loading: " << file << std::endl;
  std::cout << "mesh->polygons.size(): " << mesh->polygons.size() << std::endl;
  std::cout << "cloud->points.size(): " << cloud->points.size() << std::endl;

  // Decimate point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_small(
      new pcl::PointCloud<pcl::PointXYZ>);
  int decimate_scale = 1;
  while (cloud->points.size() / decimate_scale > decimate_thre)
    {
      decimate_scale++;
    }
  if (cloud->points.size() <= decimate_thre)
    {
      pcl::copyPointCloud(*cloud, *cloud_small);
    }
  else
    {
      for (size_t i = 0; i < cloud->points.size(); ++i)
        {
          if (i % decimate_scale == 0) {
            cloud_small->push_back(cloud->points[i]);
          }
        }
    }
  std::cout << "cloud_small->points.size(): " << cloud_small->points.size() <<
    std::endl;

  // Estimate normal
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(
      new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(
      new pcl::PointCloud<pcl::Normal>);
  tree1->setInputCloud(cloud_small);
  ne.setInputCloud(cloud_small);
  ne.setSearchMethod(tree1);
  ne.setKSearch(20);
  std::cout << "Start estimating normal..." << std::endl;
  ne.compute(*normals);
  std::cout << "Finished estimating normal." << std::endl;

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud_small, *normals, *cloud_with_normals);

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud(cloud_with_normals);

  // Apply marching cubes algorithm
  pcl::MarchingCubesRBF<pcl::PointNormal> mc;
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
  mc.setInputCloud(cloud_with_normals);
  mc.setIsoLevel(iso_level);
  mc.setGridResolution(grid_res, grid_res, grid_res);
  mc.setSearchMethod(tree);
  std::cout << "Start marching cubes reconstruction..." << std::endl;
  clock_t start = clock();
  mc.reconstruct(*triangles);
  clock_t end = clock();
  std::cout << "Reconstruction finished!" << std::endl;
  std::cout << triangles->polygons.size() << " triangles created." <<
    std::endl << "Elapsed time on CPU: " <<
    static_cast<double>(end - start) / CLOCKS_PER_SEC << " [sec]" <<
    std::endl << std::endl;

  // Save Mesh as OBJ format
  namespace fs = boost::filesystem;
  fs::path p(output);
  if (! fs::is_directory(p.parent_path()))
    {
      boost::system::error_code error;
      const bool result = fs::create_directories(p.parent_path(), error);
      if (! result || error)
        {
        std::cerr << "[ERROR] Could not create directory." << std::endl;
        }
    }
  pcl::io::saveOBJFile(output, *triangles);

  // View reconstructed mesh
  pcl::visualization::PCLVisualizer viewer("viewer");
  viewer.setBackgroundColor(0.2, 0.2, 0.2);
  // viewer.addPolygonMesh(*mesh, "mesh", 0);
  // viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", 0);
  viewer.addPolygonMesh(*triangles, "triangles", 0);
  viewer.addCoordinateSystem(0.1);
  viewer.initCameraParameters();
  viewer.setCameraPosition(
      /*pos=*/0.0, 0.0, -0.8, /*view=*/0.0, 0.0, 0.0, /*up=*/0.0, -1.0, 0.0,
      /*viewport=*/0);
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return(0);
}
