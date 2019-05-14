#include "projection_params.h"
#include "cloud_projection.h"
#include "pcl_point_types.h"
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>

using namespace cloud_to_image;

template <class T>
static void DoNotFree(T*) {}		

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	//loads projection parameters from config file
	ProjectionParams proj_params;
	std::string filename = std::string("test_files/projection_params.yaml");
	proj_params.loadFromFile(filename);

	//creates the cloud projection for HDL-64
	CloudProjection proj(*proj_params["HDL-64"]);
	std::cout << "cloud projection dimensions (rows:cols): " << proj.rows() << " : " << proj.cols() << std::endl;

	//open a depth image from file (either argument or fixed)
	if (argc < 2) {
		filename = std::string("test_files/scan00197.png");	
	} else {
		filename = std::string(argv[1]);
	}
	cv::Mat depth_image = CloudProjection::cvMatFromDepthPNG(filename);
	std::cout << "image dimensions (rows:cols): " << depth_image.rows << " : " << depth_image.cols << std::endl;
	//filename = std::string("test_image.png");
	//CloudProjection::cvMatToDepthPNG(depth_image, filename);

	//get the 3D pointcloud from depth image
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = proj.fromImage(depth_image);
	std::cout << "cloud num. points: " << cloud->points.size() << std::endl;
	//save the PCD file
	filename = std::string("cloud_from_depth_image.pcd");
	const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr(&(*cloud), &DoNotFree< pcl::PointCloud<pcl::PointXYZI> >);
	CloudProjection::cloudToPCDFile(cloud_ptr, filename);

	//creates a new cloud projection for HDL-64
	CloudProjection proj2(*proj_params["HDL-64"]);
	std::cout << "cloud projection dimensions (rows:cols): " << proj2.rows() << " : " << proj2.cols() << std::endl;

	//initialize depth image from cloud
	proj2.initFromPoints(cloud_ptr);

	//save the generated depth image
	filename = std::string("depth_image_from_cloud.png");
	CloudProjection::cvMatToDepthPNG(proj2.depth_image(), filename);


	filename = std::string("test_files/points_raw_sample.pcd");
	pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZIR>);
	if (pcl::io::loadPCDFile<pcl::PointXYZIR> (filename, *cloud2) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read pcd file \n");
	  return (-1);
	}
	const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr cloud_ptr2(&(*cloud2), &DoNotFree< pcl::PointCloud<pcl::PointXYZIR> >);

	//creates a new cloud projection for HDL-64
	CloudProjection proj3(*proj_params["HDL-64"]);
	std::cout << "cloud projection dimensions (rows:cols): " << proj3.rows() << " : " << proj3.cols() << std::endl;

	//initialize depth image from cloud
	proj3.initFromPoints(cloud_ptr2);

	//save the generated depth image
	filename = std::string("depth_image_from_points_raw.png");
	CloudProjection::cvMatToDepthPNG(proj3.depth_image(), filename);


	return 0;
}