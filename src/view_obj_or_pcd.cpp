// CopyRight [2018] <Yuto Uchimi>

#include <boost/program_options.hpp>
#include <iostream>
#include <pcl/console/print.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

int main(int argc, char** argv)
{
  std::cout << std::endl;

  std::string file;

  namespace po = boost::program_options;
  po::options_description desc("option");
  desc.add_options()
    ("file,f", po::value<std::string>(&file)->required(), "The loaded file name");

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
        std::cerr << "[ERROR] Couldn't load OBJ or PCD file. Specified file name: " <<
          file << std::endl << std::endl;
        exit(1);
      }
  }
  std::cout << "Finished loading: " << file << std::endl;


  // View reconstructed mesh
  pcl::visualization::PCLVisualizer viewer("viewer");
  viewer.setBackgroundColor(0.2, 0.2, 0.2);
  // viewer.addPolygonMesh(*mesh, "mesh", 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", 0);
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
