
#ifndef SENSOR_PARAMS_H_
#define SENSOR_PARAMS_H_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include "angles.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <yaml-cpp/yaml.h>

using boost::shared_ptr;
using boost::make_shared;

// This work was inspired on projection_params from I. Bogoslavskyi, C. Stachniss, University of Bonn 
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

namespace cloud_to_image 
{


/**
 * @brief Class for sensor specific projection parameters.
 */
class SensorParams 
{
  public:
    enum class ScanDirection { CLOCK_WISE, COUNTER_CLOCK_WISE };

    using Ptr = shared_ptr<SensorParams>;
    using ConstPtr = const shared_ptr<const SensorParams>;

    enum class Set { COLS, ROWS };
    SensorParams(): _scan_direction(ScanDirection::COUNTER_CLOCK_WISE) {}
    ~SensorParams() {}

    /**
     * @brief      Set the angle span in a given direction.
     *
     * @param[in]  span_params  The span parameters packad into ::AngularRange.
     * @param[in]  direction    The direction. Must be one of
     *                          AngularRange::Direction.
     */
    void setSpan(const AngularRange& span_params, const AngularRange::Direction& direction);

    /**
     * @brief      Set the angle spans in a given direction.
     *
     * @param[in]  span_params  The span parameters packad into ::AngularRange
     * @param[in]  direction    The direction. Must be one of
     *                          AngularRange::Direction.
     */
    void setSpan(const std::vector<AngularRange>& span_params, const AngularRange::Direction& direction);

    inline const Angle& v_start_angle() const { return _v_span_params.start_angle(); }
    
    inline const Angle& v_end_angle() const { return _v_span_params.end_angle(); }
    
    inline const Angle& v_span() const { return _v_span_params.span(); }

    inline const Angle& h_start_angle() const { return _h_span_params.start_angle(); }
    
    inline const Angle& h_end_angle() const { return _h_span_params.end_angle(); }
    
    inline const Angle& h_span() const { return _h_span_params.span(); }

    inline const AngularRange& v_span_params() const { return _v_span_params; }

    inline const AngularRange& h_span_params() const { return _h_span_params; }

    inline size_t rows() const { return _row_angles.size(); }

    inline size_t cols() const { return _col_angles.size(); }
    
    inline size_t size() const { return rows() * cols(); }

    inline const std::vector<Angle>& rowAngles() const { return _row_angles; }
    inline const std::vector<Angle>& colAngles() const { return _col_angles; }

    friend YAML::Emitter& operator << (YAML::Emitter& out, const SensorParams& val)
	{
		out << YAML::BeginMap;
		out << YAML::Key << "vertical_span";
		out << YAML::Value << val.v_span_params();
		out << YAML::Key << "horizontal_span";
		out << YAML::Value << val.h_span_params();
        out << YAML::Key << "scan_direction";
        out << YAML::Value << val.getScanDirectionStr();
		out << YAML::EndMap;
		return out;
	}

	friend std::ostream& operator << (std::ostream& out, const SensorParams& val)
	{
		out << "vertical_span: " << std::endl;
		out << val.v_span_params();
		out << "horizontal_span: " << std::endl;
		out << val.h_span_params();
        out << "scan_direction: " << std::endl; 
        out << val.getScanDirectionStr();
		return out;
	}

    /**
     * @brief      Get angle from row
     *
     * @param[in]  row   The row
     *
     * @return     Angle in radians
     */
    const Angle angleFromRow(int row) const;

    /**
     * @brief      Get angle from col
     *
     * @param[in]  col   The col
     *
     * @return     Angle in radians
     */
    const Angle angleFromCol(int col) const;

    /**
     * @brief      Get row number from angle
     *
     * @param[in]  angle  The angle
     *
     * @return     Row number
     */
    size_t rowFromAngle(const Angle& angle) const;

    /**
     * @brief      Get col number from angle
     *
     * @param[in]  angle  The angle
     *
     * @return     Col number
     */
    size_t colFromAngle(const Angle& angle) const;

    const std::vector<float>& rowAngleCosines() const { return _row_angles_cosines; }
	const std::vector<float>& colAngleCosines() const { return _col_angles_cosines; }
	const std::vector<float>& rowAngleSines() const { return _row_angles_sines; }
	const std::vector<float>& colAngleSines() const { return _col_angles_sines; }

    bool valid();

    inline void setScanDirection(const ScanDirection& direction) { _scan_direction = direction; }
    inline void setScanDirection(const std::string& direction) 
    { 
        if (direction == "CW") {
            _scan_direction = ScanDirection::CLOCK_WISE;
        } else {
            _scan_direction = ScanDirection::COUNTER_CLOCK_WISE;
        }
    }

    std::string getScanDirectionStr() const 
    {
        std::string dir = std::string("CCW");
        if (_scan_direction == ScanDirection::CLOCK_WISE) {
            dir = std::string("CW");
        }
        return dir;
    }

    const ScanDirection& getScanDirection() const { return _scan_direction; }


    /**
     * @brief      Default parameters for 16 beam Velodyne
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> VLP_16();
    /**
     * @brief      Default parameters for 16 beam Velodyne
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> VLP_32();
    /**
     * @brief      Default parameters for 32 beam Velodyne
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> HDL_32();
    /**
     * @brief      Default parameters for 64 beam Velodyne
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> HDL_64_S2();
    /**
     * @brief      Default parameters for 64 beam Velodyne
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> HDL_64_S3();
    /**
     * @brief      Parameters for 64 beam velodyne assuming equal spacing between
     *             the lasers.
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> HDL_64_EQUAL();
    /**
     * @brief      Parameters for 64 beam velodyne assuming equal spacing between
     *             the lasers.
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> HDL_64_GENERAL();
    /**
     * @brief      Parameters for 16 beam ouster
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> OS_1_16_0512();
    /**
     * @brief      Parameters for 16 beam ouster
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> OS_1_16_1024();
    /**
     * @brief      Parameters for 16 beam ouster
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> OS_1_16_2048();
    /**
     * @brief      Parameters for 64 beam ouster
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> OS_1_64_0512();
    /**
     * @brief      Parameters for 64 beam ouster
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> OS_1_64_1024();
    /**
     * @brief      Parameters for 64 beam ouster
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> OS_1_64_2048();

    /**    
     * @brief      Default parameters to cover full sphere
     *
     * @return     A pointer to parameters
     */
    static std::unique_ptr<SensorParams> fullSphere(const Angle& discretization = 5_deg);


  private:
    std::vector<Angle> fillVector(const AngularRange& span_params);
    std::vector<Angle> fillVector(const std::vector<AngularRange>& span_params);

    static size_t findClosest(const std::vector<Angle>& vec, const Angle& val);

    void fillCosSin();

    AngularRange _v_span_params;
    AngularRange _h_span_params;
    ScanDirection _scan_direction;

    std::vector<Angle> _col_angles;
    std::vector<Angle> _row_angles;

    std::vector<float> _col_angles_sines;
    std::vector<float> _col_angles_cosines;

    std::vector<float> _row_angles_sines;
    std::vector<float> _row_angles_cosines;
};

}  // namespace cloud_to_image

namespace YAML
{
	template<>
	struct convert<cloud_to_image::SensorParams> 
	{
		static Node encode(const cloud_to_image::SensorParams& rhs)
		{
			Node node;
			node["vertical_span"] = rhs.v_span_params();
			node["horizontal_span"] = rhs.h_span_params();
            node["scan_direction"] = rhs.getScanDirectionStr();
			return node;
		}

		static bool decode(const Node& node, cloud_to_image::SensorParams& rhs)
		{
			if (!node.IsMap() || node.size() != 3) {
				return false;
			}
			rhs = cloud_to_image::SensorParams();
			rhs.setSpan(node["vertical_span"].as<cloud_to_image::AngularRange>(), cloud_to_image::AngularRange::Direction::VERTICAL);
			rhs.setSpan(node["horizontal_span"].as<cloud_to_image::AngularRange>(), cloud_to_image::AngularRange::Direction::HORIZONTAL);
            rhs.setScanDirection(node["scan_direction"].as<std::string>());
			return true;
		}
	};
}

#endif  // SENSOR_PARAMS_H_
