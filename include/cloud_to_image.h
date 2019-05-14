
#ifndef CLOUD_TO_IMAGE_H_
#define CLOUD_TO_IMAGE_H_

#include "projection_params.h"
#include "cloud_projection.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace cloud_to_image
{
class CloudToImage
{
	public:
		enum class ImageOutputMode { SINGLE, GROUP, STACK, ALL };

		explicit CloudToImage();
		~CloudToImage();

		void init(int argc, char* argv[]);
		void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
		void publishImages(const std_msgs::Header& header);
		void saveImages(const std::string& base_name = std::string("cloud2image"));

	private:
		//!
		//! \brief getParam Get parameter from node handle
		//! \param nh The nodehandle
		//! \param param_name Key string
		//! \param default_value Default value if not found
		//! \return The parameter value
		//!
		template <typename T>
		T getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value);

		template <class T>
		static void DoNotFree(T*) {}

		template<class T>
		static std::string timeToStr(T ros_t);

	private:
		CloudProjection* _cloud_proj;

		std::string _cloud_topic;
		std::string _sensor_model;
		std::string _point_type;
		std::string _depth_image_topic;
		std::string _intensity_image_topic;
		std::string _reflectance_image_topic;
		std::string _noise_image_topic;

		bool _has_depth_image;
		bool _has_intensity_image;
		bool _has_reflectance_image;
		bool _has_noise_image;

		bool _save_images;
		bool _8bpp;
		bool _equalize;
		bool _flip;

		float _horizontal_scale;
		float _vertical_scale;

		unsigned int _overlapping;

		ImageOutputMode _output_mode;

		cv::Mat _depth_image;
		cv::Mat _intensity_image;
		cv::Mat _reflectance_image;
		cv::Mat _noise_image;
		cv::Mat _group_image;
		cv::Mat _stack_image;
		
		ros::NodeHandle _nodehandle;
		ros::Subscriber _sub_PointCloud;
		ros::Publisher _pub_DepthImage;
		ros::Publisher _pub_IntensityImage;
		ros::Publisher _pub_ReflectanceImage;
		ros::Publisher _pub_NoiseImage;
		ros::Publisher _pub_GroupImage;
		ros::Publisher _pub_StackImage;
};
}

#endif // CLOUD_TO_IMAGE_H_
