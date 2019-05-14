#include "cloud_to_image.h"
#include "projection_params.h"
#include "cloud_projection.h"
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/algorithm/string.hpp>

namespace cloud_to_image
{

CloudToImage::CloudToImage():
	_has_depth_image(false),
	_has_intensity_image(false),
	_has_reflectance_image(false),
	_has_noise_image(false),
	_save_images(false),
	_8bpp(false),
	_equalize(false),
	_flip(false),
	_overlapping(0),
	_output_mode(ImageOutputMode::SINGLE)
{
  _depth_image = cv::Mat::zeros(1, 1, CV_32FC1);
  _intensity_image = cv::Mat::zeros(1, 1, CV_16UC1);
  _reflectance_image = cv::Mat::zeros(1, 1, CV_16UC1);
  _noise_image = cv::Mat::zeros(1, 1, CV_16UC1);	
  _group_image = cv::Mat::zeros(1, 1, CV_16UC1);
  _stack_image = cv::Mat::zeros(1, 1, CV_16UC1);
}

CloudToImage::~CloudToImage() 
{
	if (_cloud_proj) {
		delete _cloud_proj;
	}
	_nodehandle.deleteParam("/cloud2image/cloud_topic");
	_nodehandle.deleteParam("/cloud2image/proj_params");
	_nodehandle.deleteParam("/cloud2image/sensor_model");
	_nodehandle.deleteParam("/cloud2image/point_type");

	_nodehandle.deleteParam("/cloud2image/depth_image_topic");
	_nodehandle.deleteParam("/cloud2image/intensity_image_topic");
	_nodehandle.deleteParam("/cloud2image/reflectance_image_topic");
	_nodehandle.deleteParam("/cloud2image/noise_image_topic");
	
	_nodehandle.deleteParam("/cloud2image/equalize");
}

template <typename T>
T CloudToImage::getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
{
	T value;
	if (nh.hasParam(param_name))
  	{
		nh.getParam(param_name, value);
  	}
  	else
  	{
		ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
		value = default_value;
  	}
  	return value;
}

void CloudToImage::init(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	std::string config_filename;
	_nodehandle = ros::NodeHandle("~");

	//read all parameters
	config_filename = getParam(_nodehandle, "/cloud2image/proj_params", std::string(""));

	_cloud_topic = getParam(_nodehandle, "/cloud2image/cloud_topic", std::string("/points_raw"));
	_sensor_model = getParam(_nodehandle, "/cloud2image/sensor_model", std::string("VLP-16"));
	_point_type = getParam(_nodehandle, "/cloud2image/point_type", std::string("XYZI"));

	_depth_image_topic = getParam(_nodehandle, "/cloud2image/depth_image_topic", std::string("/c2i_depth_image"));
	_intensity_image_topic = getParam(_nodehandle, "/cloud2image/intensity_image_topic", std::string("/c2i_intensity_image"));
	_reflectance_image_topic = getParam(_nodehandle, "/cloud2image/reflectance_image_topic", std::string("/c2i_reflectance_image"));
	_noise_image_topic = getParam(_nodehandle, "/cloud2image/noise_image_topic", std::string("/c2i_noise_image"));

	_horizontal_scale = getParam(_nodehandle, "/cloud2image/h_scale", 1.0);
	_vertical_scale = getParam(_nodehandle, "/cloud2image/v_scale", 1.0);
	std::string output_mode = getParam(_nodehandle, "/cloud2image/output_mode", std::string("SINGLE"));

	_save_images = getParam(_nodehandle, "/cloud2image/save_images", false);
	_overlapping = getParam(_nodehandle, "/cloud2image/overlapping", 0);

	_8bpp = getParam(_nodehandle, "/cloud2image/8bpp", false);	
	_equalize = getParam(_nodehandle, "/cloud2image/equalize", false);
	_flip = getParam(_nodehandle, "/cloud2image/flip", false);
	
	_equalize = _equalize && _8bpp; //no equalization for non 8bpp images

	if (boost::iequals(output_mode, "SINGLE")) {
		_output_mode = ImageOutputMode::SINGLE;
	} else if (boost::iequals(output_mode, "GROUP")) {
		_output_mode = ImageOutputMode::GROUP;
	} else if (boost::iequals(output_mode, "STACK")) {
		_output_mode = ImageOutputMode::STACK;
	} else if (boost::iequals(output_mode, "ALL")) {
		_output_mode = ImageOutputMode::ALL;
	} else {
		throw std::runtime_error("image output mode \"" + output_mode + "\" not supported");
	}


	if (boost::iequals(_point_type, "XYZ")) {  //no intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_output_mode = ImageOutputMode::SINGLE; //only depth images, therefore only SINGLE mode
	} else if (boost::iequals(_point_type, "XYZI")) { //with intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
	} else if (boost::iequals(_point_type, "XYZIR")) { //with intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
	} else if (boost::iequals(_point_type, "XYZIF")) { //with intensity and reflectance (ex. Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
		_has_reflectance_image = true;
	} else if (boost::iequals(_point_type, "XYZIFN")) { //with intensity and reflectance and noise (ex. Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
		_has_reflectance_image = true;
		_has_noise_image = true;
	} else {
		throw std::runtime_error("point type \"" + _point_type + "\" not supported");
	}

	//verifies the configuration file
	std::ifstream config(config_filename.c_str());
	if (!config.good()) {
		throw std::runtime_error("Projection parameters file \"" + config_filename + "\" does not exist or invalid path");	
	}

	//loads projection parameters from config file
	ProjectionParams proj_params;
	proj_params.loadFromFile(config_filename);
	//verifies the sensor model
	if (!proj_params.sensorExists(_sensor_model)) {
		throw std::runtime_error("Sensor model \"" + _sensor_model + "\" does not exist in configuration file");	
	}
	//creates the cloud projection for the sensor model
	_cloud_proj = new CloudProjection(*proj_params[_sensor_model]);


	//Mossman corrections seem to help on Velodyne
	if (boost::iequals(_sensor_model, "HDL-64") || boost::iequals(_sensor_model, "HDL-32") || boost::iequals(_sensor_model, "VLP-16")) {
		_cloud_proj->loadMossmanCorrections();
	}

	//create the subscribers and publishers
	_sub_PointCloud = _nodehandle.subscribe(_cloud_topic, 10, &CloudToImage::pointCloudCallback, this);
	
	//publishers
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
		_pub_GroupImage = _nodehandle.advertise<sensor_msgs::Image>(std::string("/c2i_group_image"), 10);
	} 
	if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
		_pub_StackImage = _nodehandle.advertise<sensor_msgs::Image>(std::string("/c2i_stack_image"), 10);
	} 
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {
		_pub_DepthImage = _nodehandle.advertise<sensor_msgs::Image>(_depth_image_topic, 10);
		_pub_IntensityImage = _nodehandle.advertise<sensor_msgs::Image>(_intensity_image_topic, 10);
		_pub_ReflectanceImage = _nodehandle.advertise<sensor_msgs::Image>(_reflectance_image_topic, 10);
		_pub_NoiseImage = _nodehandle.advertise<sensor_msgs::Image>(_noise_image_topic, 10);
	} 
}

void CloudToImage::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	//clear any previous projection data
	_cloud_proj->clearData();

	if (boost::iequals(_point_type, "XYZ")) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZ> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIR")) { 
    	pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIR>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIR> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZI")) { 
    	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZI>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZI> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIF")) { 
    	pcl::PointCloud<pcl::PointXYZIF>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIF>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIF> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIFN")) { 
    	pcl::PointCloud<pcl::PointXYZIFN>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIFN>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIFN> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	}

	//get a local copy of each image
	_depth_image = _cloud_proj->depth_image();
	_intensity_image = _cloud_proj->intensity_image();
	_reflectance_image = _cloud_proj->reflectance_image();
	_noise_image = _cloud_proj->noise_image();
	
	if (_overlapping) {
		cv::Mat left_roi = _depth_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_depth_image, left_roi, _depth_image);
		left_roi = _intensity_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_intensity_image, left_roi, _intensity_image);
		left_roi = _reflectance_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_reflectance_image, left_roi, _reflectance_image);
		left_roi = _noise_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_noise_image, left_roi, _noise_image);
	}

	publishImages(input->header);
}


void CloudToImage::publishImages(const std_msgs::Header& header)
{
	auto mode = CV_16UC1;
	auto min_range = 0.0;
	auto max_range = 65535.0;
	auto encoding = sensor_msgs::image_encodings::MONO16;
	if (_8bpp) {
		mode = CV_8UC1;
		max_range = 255;
		encoding = sensor_msgs::image_encodings::MONO8;
	}
	
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {

		if (_has_depth_image && _pub_DepthImage.getNumSubscribers() > 0) {
			//depth image is float, normalize it for visualization
			cv::Mat mono_img = cv::Mat(_depth_image.size(), mode);
			cv::normalize(_depth_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::ImagePtr depth_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_DepthImage.publish(depth_img);
		}
		if (_has_intensity_image && _pub_IntensityImage.getNumSubscribers() > 0) {

			cv::Mat mono_img = cv::Mat(_intensity_image.size(), mode);
			cv::normalize(_intensity_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::ImagePtr intensity_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_IntensityImage.publish(intensity_img);
		}
		if (_has_reflectance_image && _pub_ReflectanceImage.getNumSubscribers() > 0) {
			cv::Mat mono_img = cv::Mat(_reflectance_image.size(), mode);
			cv::normalize(_reflectance_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::ImagePtr reflectance_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_ReflectanceImage.publish(reflectance_img);
		}
		if (_has_noise_image && _pub_NoiseImage.getNumSubscribers() > 0) {
			cv::Mat mono_img = cv::Mat(_noise_image.size(), mode);
			cv::normalize(_noise_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::ImagePtr noise_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_NoiseImage.publish(noise_img);
		}
	} 
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
		//depth
		cv::Mat mono_img_depth = cv::Mat(_depth_image.size(), mode);
		cv::normalize(_depth_image, mono_img_depth, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_depth, mono_img_depth, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_depth, mono_img_depth );
		if (_flip) cv::flip( mono_img_depth, mono_img_depth, 1 );
		//intensity
		cv::Mat mono_img_intensity = cv::Mat(_intensity_image.size(), mode);
		cv::normalize(_intensity_image, mono_img_intensity, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_intensity, mono_img_intensity, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_intensity, mono_img_intensity );
		if (_flip) cv::flip( mono_img_intensity, mono_img_intensity, 1 );
		//reflectance
		cv::Mat mono_img_reflectance = cv::Mat(_reflectance_image.size(), mode);
		cv::normalize(_reflectance_image, mono_img_reflectance, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_reflectance, mono_img_reflectance, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_reflectance, mono_img_reflectance );
		if (_flip) cv::flip( mono_img_reflectance, mono_img_reflectance, 1 );
		//noise
		cv::Mat mono_img_noise = cv::Mat(_noise_image.size(), mode);
		cv::normalize(_noise_image, mono_img_noise, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_noise, mono_img_noise, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_noise, mono_img_noise );
		if (_flip) cv::flip( mono_img_noise, mono_img_noise, 1 );

		//count the effective number of images to output
		int img_count = 0;
		img_count += _has_depth_image;
		img_count += _has_intensity_image;
		img_count += _has_reflectance_image;
		img_count += _has_noise_image;

		if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
			//prerequisite: all above images have same width and height
			auto group_size = mono_img_depth.size();
			group_size.height = group_size.height * img_count; 
			cv::Mat mono_img_group = cv::Mat(group_size, mode);

			auto cols = mono_img_depth.cols;
			auto rows = mono_img_depth.rows;
			img_count = 0;
			if (_has_depth_image) {
				//copy depth image
				mono_img_depth.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			if (_has_intensity_image) {
				//copy intensity image
				mono_img_intensity.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			if (_has_reflectance_image) {
				//copy reflectance image
				mono_img_reflectance.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			if (_has_noise_image) {
				//copy noise image
				mono_img_noise.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			//get a local copy
			_group_image = mono_img_group;

			//publish
			sensor_msgs::ImagePtr output_img = cv_bridge::CvImage(header, encoding, mono_img_group).toImageMsg();
			_pub_GroupImage.publish(output_img);

		} 
		if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
			//prerequisite: all above images have same width and height
			auto format = CV_16UC1;
			auto encoding = sensor_msgs::image_encodings::MONO16;
			if (_8bpp) {
				format = CV_8UC1;
				encoding = sensor_msgs::image_encodings::MONO8;
			}
			if (img_count == 2) {
				format = CV_16UC2;
				encoding = sensor_msgs::image_encodings::RGB16;
				if (_8bpp) {
					format = CV_8UC2;
					encoding = sensor_msgs::image_encodings::RGB8;
				}
			} else if (img_count == 3) {
				format = CV_16UC3;
				encoding = sensor_msgs::image_encodings::RGB16;
				if (_8bpp) {
					format = CV_8UC3;
					encoding = sensor_msgs::image_encodings::RGB8;
				}
			} else if (img_count == 4) {
				format = CV_16UC4;
				encoding = sensor_msgs::image_encodings::RGBA16;
				if (_8bpp) {
					format = CV_8UC4;
					encoding = sensor_msgs::image_encodings::RGBA8;
				}
			}
			cv::Mat mono_img_stack = cv::Mat(mono_img_depth.size(), format); //n channels

			//special case: if only two images, add space for the 3rd channel
			if (img_count == 2) {
				img_count++;
			}
			std::vector<cv::Mat> images(img_count);
			img_count = 0;

			if (_has_depth_image) {
				images.at(img_count++) = mono_img_depth; 
			}
			if (_has_intensity_image) {
				images.at(img_count++) = mono_img_intensity; 
			}
			if (_has_reflectance_image) {
				images.at(img_count++) = mono_img_reflectance;
			}
			if (_has_noise_image) {
				images.at(img_count++) = mono_img_noise;
			}

			//special case: if only two images, fill with zeros for the 3rd channel
			if (img_count == 2) {
				images.at(img_count) = cv::Mat::zeros(mono_img_depth.rows, mono_img_depth.cols, mode);;	
			}

			//combine all images as separate channels
			cv::merge(images, mono_img_stack);

			//get a local copy
			_stack_image = mono_img_stack;

			//publish
			sensor_msgs::ImagePtr output_img = cv_bridge::CvImage(header, encoding, mono_img_stack).toImageMsg();
			_pub_StackImage.publish(output_img);
		}
	}

	//output images to PNG files if so requested
	if (_save_images) {
		saveImages();
	}
}

template<class T>
std::string CloudToImage::timeToStr(T ros_t)
{
    (void)ros_t;
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S.%f");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

void CloudToImage::saveImages(const std::string& base_name)
{
	std::string filename;
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {
		if (_has_depth_image) {
			//save the generated depth image
			filename = std::string(base_name + "_depth");
			filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png");
			CloudProjection::cvMatToDepthPNG(_depth_image, filename);
		}
		if (_has_intensity_image) {
			//save the generated intensity image
			filename = std::string(base_name + "_intensity");
			filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png");
			CloudProjection::cvMatToDepthPNG(_intensity_image, filename);
		}

		if (_has_reflectance_image) {
			//save the generated reflectance image
			filename = std::string(base_name + "_reflectance");
			filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png");
			CloudProjection::cvMatToDepthPNG(_reflectance_image, filename);
		}

		if (_has_noise_image) {
			//save the generated noise image
			filename = std::string(base_name + "_noise");
			filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png");
			CloudProjection::cvMatToDepthPNG(_noise_image, filename);
		}
	} 
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
		//save the generated group image
		filename = std::string(base_name + "_group");
		filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png");
		CloudProjection::cvMatToDepthPNG(_group_image, filename);
	} 
	if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
		//save the generated group image
		filename = std::string(base_name + "_stack");
		filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png");
		CloudProjection::cvMatToColorPNG(_stack_image, filename);
	}
}

}
