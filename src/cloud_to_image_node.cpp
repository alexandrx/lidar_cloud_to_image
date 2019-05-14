#include "cloud_to_image.h"
#include <ros/ros.h>

using namespace cloud_to_image;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cloud2image");

	CloudToImage cld2img;
	cld2img.init(argc, argv);

	ros::spin();

	return 0;
}