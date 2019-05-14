
#include "cloud_projection.h"
#include "pcl_point_types.h"
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

// This work was inspired on cloud_projection from I. Bogoslavskyi, C. Stachniss, University of Bonn 
// https://github.com/PRBonn/cloud_to_image.git
// The original license copyright is as follows:
//------------------------------------
// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------

namespace cloud_to_image {

const std::vector<float> MOOSMAN_CORRECTIONS{
    {0.02587499999999987,   -0.0061250000000001581, 0.031874999999999876,
     0.001874999999999849,  0.029874999999999874,   -0.1961250000000001,
     0.049874999999999892,  -0.034125000000000183,  0.0038749999999998508,
     0.0058749999999998526, 0.035874999999999879,   -0.064124999999999988,
     0.035874999999999879,  0.001874999999999849,   -0.024125000000000174,
     -0.062124999999999986, 0.039874999999999883,   -0.020125000000000171,
     0.075874999999999915,  -0.024125000000000174,  -0.0041250000000001563,
     -0.058124999999999982, -0.032125000000000181,  -0.058124999999999982,
     0.021874999999999867,  -0.032125000000000181,  0.059874999999999901,
     -0.04412499999999997,  0.075874999999999915,   -0.0041250000000001563,
     0.021874999999999867,  0.0058749999999998526,  -0.036125000000000185,
     -0.022125000000000172, -0.0041250000000001563, -0.058124999999999982,
     -0.026125000000000176, -0.030125000000000179,  0.045874999999999888,
     0.035874999999999879,  -0.026125000000000176,  0.041874999999999885,
     -0.086125000000000007, -0.060124999999999984,  0.031874999999999876,
     -0.010125000000000162, -0.024125000000000174,  -0.048124999999999973,
     -0.038125000000000187, 0.039874999999999883,   -0.026125000000000176,
     0.037874999999999881,  -0.020125000000000171,  0.051874999999999893,
     -0.014125000000000165, 0.019874999999999865,   -0.0021250000000001545,
     0.027874999999999872,  0.0058749999999998526,  0.021874999999999867,
     0.023874999999999869,  0.085874999999999702,   0.085874999999999702,
     0.11587499999999995}};

CloudProjection::CloudProjection(const SensorParams& params)
    : _params(params)
{
  if (!_params.valid()) {
    throw std::runtime_error("sensor parameters not valid for projection.");
  }
  clearData();
  clearCorrections();
}

void CloudProjection::clearData() 
{
  _data = PointMatrix(_params.cols(), PointColumn(_params.rows()));
  _depth_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_32FC1);
  _intensity_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);
  _reflectance_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);
  _noise_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);
}

void CloudProjection::loadMossmanCorrections()
{
  _corrections = MOOSMAN_CORRECTIONS;
}

void CloudProjection::clearCorrections()
{
  _corrections.clear();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);    
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZI>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    uint16_t intensity = (uint16_t)((point.intensity/255.0)*65535.0);
    
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    //std::cout << point.x << "," << point.y << "," << point.z << "," << dist_to_sensor << "," << angle_rows.toDegrees() << "," << angle_cols.toDegrees() << "," << bin_rows << "," << bin_cols << "," << std::endl;
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIR>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    uint16_t intensity = (uint16_t)((point.intensity/255.0)*65535.0);
    uint16_t ring = point.ring; //but ring is ignored here
    (void)ring;
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIF>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    uint16_t intensity = point.intensity;
    uint16_t reflectivity = point.reflectivity;
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    auto& current_written_reflectivity = this->_reflectance_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      current_written_reflectivity = reflectivity;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    uint16_t intensity = point.intensity;
    uint16_t reflectivity = point.reflectivity;
    uint16_t noise = point.noise;
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    auto& current_written_reflectivity = this->_reflectance_image.template at<uint16_t>(bin_rows, bin_cols);
    auto& current_written_noise = this->_noise_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      current_written_reflectivity = reflectivity;
      current_written_noise = noise;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::fromImage(const cv::Mat& depth_image) {
  checkImageAndStorage(depth_image);
  cloneDepthImage(depth_image);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (int r = 0; r < depth_image.rows; ++r) {
    for (int c = 0; c < depth_image.cols; ++c) {
      if (depth_image.at<float>(r, c) < 0.0001f) {
        continue;
      }
      pcl::PointXYZ point;
      unprojectPoint(depth_image, r, c, point);
      pcl::PointXYZI point2;
      point2.x = point.x;
      point2.y = point.y;
      point2.z = point.z;
      point2.intensity = 0;
      cloud->points.push_back(point2);
      this->at(r, c).points().push_back(cloud->points.size() - 1);
    }
  }
  return cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::fromImage(const cv::Mat& depth_image, const cv::Mat& intensity_image) 
{
  checkImageAndStorage(depth_image);
  cloneDepthImage(depth_image);
  checkImageAndStorage(intensity_image);
  cloneDepthImage(intensity_image);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (int r = 0; r < depth_image.rows; ++r) {
    for (int c = 0; c < depth_image.cols; ++c) {
      if (depth_image.at<float>(r, c) < 0.0001f) {
        continue;
      }
      pcl::PointXYZI point;
      unprojectPoint(depth_image, intensity_image, r, c, point);
      cloud->points.push_back(point);
      this->at(r, c).points().push_back(cloud->points.size() - 1);
    }
  }
  return cloud;
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const int row, const int col, pcl::PointXYZ& point) const 
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZI& point) const 
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZIR& point) const 
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
  point.ring = row; //ring is handled same as row number
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const int row, const int col, pcl::PointXYZIF& point) const
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
  point.reflectivity =  reflectance_image.at<uint16_t>(row, col);
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const cv::Mat& noise_image, const int row, const int col, pcl::PointXYZIFN& point) const
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
  point.reflectivity =  reflectance_image.at<uint16_t>(row, col);
  point.noise =  noise_image.at<uint16_t>(row, col);
}


template <typename T>
void CloudProjection::checkCloudAndStorage(const T& cloud) {
  if (this->_data.size() < 1) {
    throw std::length_error("_data size is < 1");
  }
  if (cloud->points.size()==0) {
    throw std::runtime_error("cannot fill from cloud: no points");
  }
}

void CloudProjection::checkImageAndStorage(const cv::Mat& image) {
  if (image.type() != CV_32F && image.type() != CV_16U) {
    throw std::runtime_error("wrong image format");
  }
  if (this->_data.size() < 1) {
    throw std::length_error("_data size is < 1");
  }
  if (this->rows() != static_cast<size_t>(image.rows) ||
      this->cols() != static_cast<size_t>(image.cols)) {
    throw std::length_error("_data dimensions do not correspond to image ones");
  }
}

void CloudProjection::fixDepthSystematicErrorIfNeeded() {
  if (_depth_image.rows < 1) {
    //fprintf(stderr, "[INFO]: depth image of wrong size, not correcting depth\n");
    return;
  }
  if (_intensity_image.rows < 1) {
    //fprintf(stderr, "[INFO]: intensity image of wrong size, not correcting depth\n");
    return;
  }
  if (_reflectance_image.rows < 1) {
    //fprintf(stderr, "[INFO]: reflectance image of wrong size, not correcting depth\n");
    return;
  }
  if (_noise_image.rows < 1) {
    //fprintf(stderr, "[INFO]: noise image of wrong size, not correcting depth\n");
    return;
  }  
  if (_corrections.size() != static_cast<size_t>(_depth_image.rows)) {
    //fprintf(stderr, "[INFO]: Not correcting depth data.\n");
    return;
  }
  for (int r = 0; r < _depth_image.rows; ++r) {
    auto correction = _corrections[r];
    for (int c = 0; c < _depth_image.cols; ++c) {
      if (_depth_image.at<float>(r, c) < 0.001f) {
        continue;
      }
      _depth_image.at<float>(r, c) -= correction;
    }
  }
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::readKittiCloud(const std::string& filename) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  std::fstream file(filename.c_str(), std::ios::in | std::ios::binary);
  if (file.good()) {
    file.seekg(0, std::ios::beg);
    for (int i = 0; file.good() && !file.eof(); ++i) {
      pcl::PointXYZI point;
      file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
      // ignore intensity
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
      cloud->push_back(point);
    }
    file.close();
  }
  if (!cloud->points.size()) {
    throw std::runtime_error("point cloud is empty, cannot load");
  } 
  return cloud;
}

cv::Mat CloudProjection::fixKITTIDepth(const cv::Mat& original)  
{
  cv::Mat fixed = original;
  for (int r = 0; r < fixed.rows; ++r) {
    auto correction = MOOSMAN_CORRECTIONS[r];
    for (int c = 0; c < fixed.cols; ++c) {
      if (fixed.at<float>(r, c) < 0.001f) {
        continue;
      }
      fixed.at<float>(r, c) -= correction;
    }
  }
  return fixed;
}

cv::Mat CloudProjection::cvMatFromDepthPNG(const std::string& filename) 
{
  cv::Mat depth_image = cv::imread(filename, cv::IMREAD_ANYDEPTH /*CV_LOAD_IMAGE_ANYDEPTH*/);
  depth_image.convertTo(depth_image, CV_32F);
  if (depth_image.type() != CV_32F && depth_image.type() != CV_16U) {
    throw std::runtime_error("wrong image format, cannot load");
  }
  if (depth_image.rows < 1 || depth_image.cols < 1) {
    throw std::runtime_error("wrong image format, cannot load");
  }

  depth_image /= 500.;
  return fixKITTIDepth(depth_image);
}

void CloudProjection::cloudToPCDFile(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string& filename)
{
  if (!cloud->points.size()) {
    throw std::runtime_error("point cloud is empty, cannot save");
  }  
  pcl::io::savePCDFileBinary(filename, *cloud);
}

void CloudProjection::cvMatToDepthPNG(const cv::Mat& image, const std::string& filename)
{
  if (image.type() != CV_32F && image.type() != CV_16U) {
    throw std::runtime_error("wrong image format, cannot save");
  }
  if (image.rows < 1 || image.cols < 1) {
    throw std::runtime_error("wrong image format, cannot save");;
  }
  
  try {
      cv::imwrite(filename, image);
  }
  catch (std::runtime_error& ex) {
      fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
  }
}

void CloudProjection::cvMatToColorPNG(const cv::Mat& image, const std::string& filename)
{
  if (image.rows < 1 || image.cols < 1) {
    throw std::runtime_error("wrong image format, cannot save");;
  }  
  try {
      cv::imwrite(filename, image);
  }
  catch (std::runtime_error& ex) {
      fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
  }
}


}  // namespace cloud_to_image
