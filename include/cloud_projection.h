
#ifndef CLOUD_PROJECTION_H_
#define CLOUD_PROJECTION_H_

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include <list>
#include <memory>
#include <stdexcept>
#include <vector>

#include <pcl_ros/point_cloud.h>

#include "projection_params.h"
#include "angles.h"
#include "pcl_point_types.h"

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

/**
 * @brief      Abstract class for cloud projection.
 */
class CloudProjection {
  class PointContainer;
  // some useful usings
  using PointColumn = std::vector<PointContainer>;
  using PointMatrix = std::vector<PointColumn>;

  public:
    using Ptr = shared_ptr<CloudProjection>;
    using ConstPtr = shared_ptr<const CloudProjection>;

    explicit CloudProjection(const SensorParams& params);
    virtual ~CloudProjection() {}

    void clearData();

    /**
     * @brief      Initialize from 3d points.
     *
     * @param[in]  cloud  The points
     */
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& cloud);
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr& cloud);
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr& cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr fromImage(const cv::Mat& depth_image);
    pcl::PointCloud<pcl::PointXYZI>::Ptr fromImage(const cv::Mat& depth_image, const cv::Mat& intensity_image);

    inline const cv::Mat& depth_image() const { return this->_depth_image; }

    inline cv::Mat& depth_image() { return this->_depth_image; }

    inline void cloneDepthImage(const cv::Mat& image) { _depth_image = image.clone(); }

    inline const cv::Mat& intensity_image() const { return this->_intensity_image; }

    inline cv::Mat& intensity_image() { return this->_intensity_image; }

    inline void cloneIntensityImage(const cv::Mat& image) { _intensity_image = image.clone(); }

    inline const cv::Mat& reflectance_image() const { return this->_reflectance_image; }

    inline cv::Mat& reflectance_image() { return this->_reflectance_image; }

    inline void cloneReflectanceImage(const cv::Mat& image) { _reflectance_image = image.clone(); }

    inline const cv::Mat& noise_image() const { return this->_noise_image; }

    inline cv::Mat& noise_image() { return this->_noise_image; }

    inline void cloneNoiseImage(const cv::Mat& image) { _noise_image = image.clone(); }

    inline size_t rows() const { return _params.rows(); }

    inline size_t cols() const { return _params.cols(); }

    inline size_t size() const { return _params.size(); }

    inline const SensorParams& params() const { return _params; }

    inline const PointContainer& at(const size_t row, const size_t col) const { return _data[col][row]; }

    inline PointContainer& at(const size_t row, const size_t col) { return _data[col][row]; }

    inline const PointMatrix& matrix() const { return _data; }

    /**
     * @brief      Check if where we store data is valid.
     *
     * @param[in]  image  The image to check
     */
    void checkImageAndStorage(const cv::Mat& image);

    /**
     * @brief      Check if where we store data is valid.
     *
     * @param[in]  points  The points to check
     */
    template <typename T>
    void checkCloudAndStorage(const T& points);

    /**
     * @brief      Unproject a point from depth image coordinate
     *
     * @param[in]  image  A depth image
     * @param[in]  row    A row in the image
     * @param[in]  col    A col in the image
     *
     * @return     { description_of_the_return_value }
     */
    void unprojectPoint(const cv::Mat& depth_image, const int row, const int col, pcl::PointXYZ& point) const;
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZI& point) const;
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZIR& point) const;
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const int row, const int col, pcl::PointXYZIF& point) const;
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const cv::Mat& noise_image, const int row, const int col, pcl::PointXYZIFN& point) const;


    /**
     * @brief      Set corrections for systematic error in a dataset (see
     *             notebooks in the repo)
     *
     * @param[in]  corrections  A vector of correction in depth for every beam.
     */
    inline void setCorrections(const std::vector<float>& corrections) 
    {
      _corrections = corrections;
    }

    /**
     * @brief      Fix systematic error. See notebooks in the repo for details.
     */
    void fixDepthSystematicErrorIfNeeded();

    static pcl::PointCloud<pcl::PointXYZI>::Ptr readKittiCloud(const std::string& filename);
    static cv::Mat cvMatFromDepthPNG(const std::string& filename);

    static void cloudToPCDFile(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string& filename);
    static void cvMatToDepthPNG(const cv::Mat& image, const std::string& filename);
    static void cvMatToColorPNG(const cv::Mat& image, const std::string& filename);

    void loadMossmanCorrections();
    void clearCorrections();

  private:
    static cv::Mat fixKITTIDepth(const cv::Mat& original);

  protected:
    // just stores addresses of the points. Does not own them.
    PointMatrix _data;

    SensorParams _params;

    cv::Mat _depth_image;

    cv::Mat _intensity_image;

    cv::Mat _reflectance_image;

    cv::Mat _noise_image;

    std::vector<float> _corrections;
};

/**
 * @brief      Class for point container.
 */
class CloudProjection::PointContainer {
  public:
    PointContainer() {}

    inline bool isEmpty() const { return _points.empty(); }

    inline std::list<size_t>& points() { return _points; }

    inline const std::list<size_t>& points() const { return _points; }

  private:
    std::list<size_t> _points;
};

}  // namespace cloud_to_image

#endif  // CLOUD_PROJECTION_H_
