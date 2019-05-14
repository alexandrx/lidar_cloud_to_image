#ifndef ANGLES_H
#define ANGLES_H

#include <cmath>
#include <stdio.h>
#include <limits>
#include <iostream>

#include <yaml-cpp/yaml.h>

// This work was inspired on radians.h from I. Bogoslavskyi, C. Stachniss, University of Bonn 
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
//
// This class provides cooked literals for Angles, following on the description in:
// https://akrzemi1.wordpress.com/2012/08/12/user-defined-literals-part-i/
namespace cloud_to_image 
{
  class Angle;
}

//forward declaration of literal operators
constexpr cloud_to_image::Angle operator"" _rad(long double Angle);
constexpr cloud_to_image::Angle operator"" _deg(
    unsigned long long int Angle);
constexpr cloud_to_image::Angle operator"" _deg(long double Angle);

namespace cloud_to_image
{

class Angle 
{
	public:
	    class IsAngle {};  // a tag to prevent using raw constructor
	    Angle() : _raw_angle{0}, _valid{false} {}
	    explicit constexpr Angle(IsAngle, float angle)
		: _raw_angle{angle}, _valid{true} {}

	    inline float val() const { return _raw_angle; }
	    inline bool valid() const { return _valid; }

	    static Angle fromRadians(float radians) 
	    {
	      return Angle{IsAngle{}, radians};
	    }

	    static Angle fromDegrees(float degrees) 
	    {
	      return Angle{IsAngle{}, static_cast<float>(degrees * M_PI / 180.)};
	    }

	    inline float toRadians() const { return _raw_angle; }
	    inline float toDegrees() const { return 180. * _raw_angle / M_PI; }

	    Angle operator-(const Angle& other) const 
	    {
	      return fromRadians(_raw_angle - other._raw_angle);
	    }

	    Angle operator+(const Angle& other) const 
	    {
	      return fromRadians(_raw_angle + other._raw_angle);
	    }

	    Angle operator+=(const Angle& other) 
	    {
	      this->_raw_angle += other._raw_angle;
	      return *this;
	    }

	    Angle operator-=(const Angle& other) 
	    {
	      this->_raw_angle -= other._raw_angle;
	      return *this;
	    }

	    Angle operator/(const float& num) const 
	    {
	      return fromRadians(_raw_angle / num);
	    }

	    float operator/(const Angle& other) const 
	    {
	      return _raw_angle / other._raw_angle;
	    }

	    Angle operator*(const float& num) const 
	    {
	      return fromRadians(_raw_angle * num);
	    }

	    Angle operator-() { return fromRadians(-_raw_angle); }

	    bool operator<(const Angle& other) const 
	    {
	      return _raw_angle < other._raw_angle - std::numeric_limits<float>::epsilon();
	    }

	    bool operator>(const Angle& other) const 
	    {
	      return _raw_angle > other._raw_angle + std::numeric_limits<float>::epsilon();
	    }

	    bool operator==(const Angle& other) const 
	    {
	      return std::fabs(_raw_angle - other._raw_angle) <= std::numeric_limits<float>::epsilon();
	    }

	    void normalize(const Angle& from = 0_deg, const Angle& to = 360_deg) 
	    {
	      float diff = (to - from).val();
	      while (_raw_angle < from.val()) {
			_raw_angle += diff;
	      }
	      while (_raw_angle > to.val()) {
			_raw_angle -= diff;
	      }
	    }

	    Angle normalize(const Angle& from = 0_deg, const Angle& to = 360_deg) const 
	    {
	      Angle new_Angle = fromRadians(_raw_angle);
	      new_Angle.normalize(from, to);
	      return new_Angle;
	    }

	    static Angle abs(const Angle& Angle) 
	    {
	      return Angle::fromRadians(std::fabs(Angle._raw_angle));
	    }

	    static Angle floor(const Angle& Angle) 
	    {
	      return Angle::fromDegrees(std::floor(Angle.toDegrees()));
	    }

	    friend std::ostream& operator <<(std::ostream& out, const Angle& val)
		{
			out << val.toDegrees() << std::endl;
			return out;
		}

		friend YAML::Emitter& operator <<(YAML::Emitter& out, const Angle& val)
		{
			out << val.toDegrees();
			return out;
		}

	protected:
	    float _raw_angle;
	    bool _valid;
};

class AngularRange 
{
	public:
		/**
		* Enum for the direction of the angular range.
		*/
		enum class Direction { HORIZONTAL, VERTICAL };

		AngularRange() {}
		AngularRange(const Angle& start_angle, const Angle& end_angle, const Angle& step, const std::vector<float>& correction_table = std::vector<float>()) {
			_start_angle = start_angle;
			_end_angle = end_angle;
			_step = step;
			_num_beams = std::round(std::fabs((_end_angle - _start_angle) / _step));
			_span = Angle::abs(end_angle - start_angle);
			if (correction_table.size()) {
				setAngleCorrectionTable(correction_table);
			}
		}
		AngularRange(const Angle& start_angle, const Angle& end_angle, int num_beams, const std::vector<float>& correction_table = std::vector<float>()) {
			_start_angle = start_angle;
			_end_angle = end_angle;
			_num_beams = num_beams;
			_step = Angle::abs((_end_angle - _start_angle) / _num_beams);
			_span = Angle::abs(end_angle - start_angle);
			if (correction_table.size()) {
				setAngleCorrectionTable(correction_table);
			}
		}
		AngularRange(const float start_angle, const float end_angle, const float step, const std::vector<float>& correction_table = std::vector<float>()) {
			AngularRange(Angle(Angle::IsAngle{}, start_angle), Angle(Angle::IsAngle{}, end_angle), Angle(Angle::IsAngle{}, step), correction_table);
			// _start_angle = Angle(Angle::IsAngle{}, start_angle);
			// _end_angle = Angle(Angle::IsAngle{}, end_angle);
			// _step = Angle(Angle::IsAngle{}, step);
			// _num_beams = std::round(std::fabs((_end_angle - _start_angle) / _step));
			// _span = Angle::abs(_end_angle - _start_angle);
		}
		AngularRange(const float start_angle, const float end_angle, int num_beams, const std::vector<float>& correction_table = std::vector<float>()) {
			AngularRange(Angle(Angle::IsAngle{}, start_angle), Angle(Angle::IsAngle{}, end_angle), num_beams, correction_table);
			// _start_angle = Angle(Angle::IsAngle{}, start_angle);
			// _end_angle = Angle(Angle::IsAngle{}, end_angle);
			// _num_beams = num_beams;
			// _step = Angle::abs((_end_angle - _start_angle) / _num_beams);
			// _span = Angle::abs(_end_angle - _start_angle);
		}

		AngularRange(const AngularRange& other)
		{
			*this = other;
		}

		const Angle& start_angle() const { return _start_angle; }
		const Angle& end_angle() const { return _end_angle; }
		const Angle& step() const { return _step; }
		const Angle& span() const { return _span; }
		int num_beams() const { return _num_beams; }

		bool valid() const { return _num_beams > 0 && _span > 0_deg; }

		const std::vector<Angle>& getAngleCorrectionTable() const { return _angle_correction_table; }
		const std::vector<float> getAngleCorrectionTableF() const 
		{ 
			std::vector<float> correction_table;
			correction_table.clear();
			for (auto& it : _angle_correction_table) {
				correction_table.push_back(it.toDegrees());
			}
			return correction_table; 
		}
		void setAngleCorrectionTable(const std::vector<Angle>& correction_table) { _angle_correction_table = correction_table; }
		void setAngleCorrectionTable(const std::vector<float>& correction_table) 
		{ 
			_angle_correction_table.clear();
			for (auto& it : correction_table) {
				_angle_correction_table.push_back(Angle::fromDegrees(it));
			}
		}

		AngularRange& operator = (const AngularRange& other)
		{
			if (this != &other) {
				_start_angle = other._start_angle;
				_end_angle = other._end_angle;
				_step = other._step;
				_span = other._span;
				_num_beams = other._num_beams;
				_angle_correction_table = other._angle_correction_table;
			}
			return *this;
		}

		friend YAML::Emitter& operator <<(YAML::Emitter& out, const AngularRange& val)
		{
			out << YAML::BeginMap;
			out << YAML::Key << "start_angle";
			out << YAML::Value << val.start_angle().toDegrees();
			out << YAML::Key << "end_angle";
			out << YAML::Value << val.end_angle().toDegrees();
			out << YAML::Key << "step";
			std::cout << "Emit angle table of size: " << val.getAngleCorrectionTable().size() << std::endl;
			out << YAML::Value  << val.step().toDegrees();
			if (val.getAngleCorrectionTable().size()) {
				out << YAML::Key << "angle_correction_table";
				out << YAML::Value << YAML::Flow << val.getAngleCorrectionTableF();
			}
			out << YAML::EndMap;
			return out;
		}

		friend std::ostream& operator <<(std::ostream& out, const AngularRange& val)
		{
			out << "start_angle: ";
			out << val.start_angle() << std::endl;
			out << "end_angle: ";
			out << val.end_angle() << std::endl;
			out << "step: ";
			out << val.step() << std::endl;
			out << "beams: ";
			out << val.num_beams() << std::endl;
			out << "span: ";
			out << val.span() << std::endl;
			if (val.getAngleCorrectionTable().size()) {
				out << "angle_correction_table: [";
				for (auto it = val.getAngleCorrectionTable().begin(); it != val.getAngleCorrectionTable().end(); ++it) {
					out << it->toDegrees();
					if (it < val.getAngleCorrectionTable().end()) {
						out << ",";
					}
				}
				out << "]" << std::endl;
			}
			return out;
		}

	private:
		Angle _start_angle = 0_deg;
		Angle _end_angle = 0_deg;
		Angle _step = 0_deg;
		Angle _span = 0_deg;
		int _num_beams = 0;
		std::vector<Angle> _angle_correction_table;
};


}  // namespace cloud_to_image

constexpr cloud_to_image::Angle operator"" _rad(long double Angle) 
{
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle)};
}

constexpr cloud_to_image::Angle operator"" _deg(unsigned long long int Angle) 
{
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle * M_PI / 180.0)};
}

constexpr cloud_to_image::Angle operator"" _deg(long double Angle) 
{
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle * M_PI / 180.0)};
}

namespace YAML
{
	template<>
	struct convert<cloud_to_image::Angle> 
	{
		static Node encode(const cloud_to_image::Angle& rhs)
		{
			Node node;
			node = rhs.toDegrees();
			return node;
		}

		static bool decode(const Node& node, cloud_to_image::Angle& rhs)
		{
			rhs = cloud_to_image::Angle::fromDegrees(node.as<float>());
			return true;
		}
	};

	template<>
	struct convert<cloud_to_image::AngularRange> 
	{
		static Node encode(const cloud_to_image::AngularRange& rhs)
		{
			Node node;
			node["start_angle"] = rhs.start_angle().toDegrees();
			node["end_angle"] = rhs.end_angle().toDegrees();
			node["step"] = rhs.step().toDegrees();
			if (rhs.getAngleCorrectionTable().size()) {
				node["angle_correction_table"] = rhs.getAngleCorrectionTable();
			}
			return node;
		}

		static bool decode(const Node& node, cloud_to_image::AngularRange& rhs)
		{
			if (!node.IsMap() || (node.size() != 3 && node.size() != 4)) {
				return false;
			}
			rhs = cloud_to_image::AngularRange(cloud_to_image::Angle::fromDegrees(node["start_angle"].as<float>()), 
				                               cloud_to_image::Angle::fromDegrees(node["end_angle"].as<float>()), 
				                               cloud_to_image::Angle::fromDegrees(node["step"].as<float>()));
			if (node["angle_correction_table"]) {
				rhs.setAngleCorrectionTable(node["angle_correction_table"].as< std::vector<float> >());
			}

			return true;
		}
	};
}

#endif  // ANGLES_H
