
#include "sensor_params.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

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

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

namespace cloud_to_image 
{

//using boost::algorithm::starts_with;

void SensorParams::setSpan(const AngularRange& span_params, const AngularRange::Direction& direction) 
{
  std::vector<AngularRange> params_vec = {{span_params}};
  this->setSpan(params_vec, direction);
}

void SensorParams::setSpan(const std::vector<AngularRange>& span_params, const AngularRange::Direction& direction) 
{
  int num_beams = 0;
  for (const auto& span : span_params) {
    num_beams += span.num_beams();
  }
  switch (direction) {
    case AngularRange::Direction::HORIZONTAL:
      _h_span_params = AngularRange(span_params.front().start_angle(),
                                    span_params.back().end_angle(), 
                                    num_beams, 
                                    span_params.front().getAngleCorrectionTableF());
      _col_angles = fillVector(span_params);
      break;
    case AngularRange::Direction::VERTICAL:
      _v_span_params = AngularRange(span_params.front().start_angle(),
                                   span_params.back().end_angle(), 
                                   num_beams,
                                   span_params.front().getAngleCorrectionTableF());
      _row_angles = fillVector(span_params);
      break;
  }
  fillCosSin();
}

std::vector<Angle> SensorParams::fillVector(const AngularRange& span_params) 
{
  std::vector<AngularRange> params_vec = {{span_params}};
  return this->fillVector(params_vec);
}

std::vector<Angle> SensorParams::fillVector(const std::vector<AngularRange>& span_params) 
{
  std::vector<Angle> res;
  for (const auto span_param : span_params) {
    if (span_param.getAngleCorrectionTable().size()) {
      for (auto& it : span_param.getAngleCorrectionTable()) {
        res.push_back(it);
      }
    } else {
    	int direction = 1; //increasing
      Angle rad = span_param.start_angle();
      if (span_param.start_angle() > span_param.end_angle()) {
      	direction = -1;
      }
      for (int i = 0; i < span_param.num_beams(); ++i) {
        if (rad == 0_deg) {
          rad = 0_deg;
        }
        res.push_back(rad);
        rad += (span_param.step() * direction);
      }
    }
  }
  return res;
}

bool SensorParams::valid() 
{
  bool all_params_valid = _v_span_params.valid() && _h_span_params.valid();
  bool arrays_empty = _row_angles.empty() && _col_angles.empty();
  bool cos_sin_empty = _row_angles_sines.empty() &&
                       _row_angles_cosines.empty() &&
                       _col_angles_sines.empty() && _col_angles_cosines.empty();
  if (!all_params_valid) {
    throw std::runtime_error("Sensor parameters invalid.");
  }
  if (arrays_empty) {
    throw std::runtime_error("Sensor parameters arrays not filled.");
  }
  if (cos_sin_empty) {
    throw std::runtime_error("Projection parameters sin and cos arrays not filled.");
  }
  return true;
}

const Angle SensorParams::angleFromRow(int row) const 
{
  if (row >= 0 && static_cast<size_t>(row) < _row_angles.size()) {
    return _row_angles[row];
  }
  fprintf(stderr, "ERROR: row %d is wrong\n", row);
  return 0.0_deg;
}

const Angle SensorParams::angleFromCol(int col) const 
{
  int actual_col = col;
  if (col < 0) {
    actual_col = col + _col_angles.size();
  } else if (static_cast<size_t>(col) >= _col_angles.size()) {
    actual_col = col - _col_angles.size();
  }
  // everything is normal
  return _col_angles[actual_col];
}

size_t SensorParams::rowFromAngle(const Angle& angle) const 
{
  size_t row = findClosest(_row_angles, angle);
  //std::cout << angle << " -> " << row << std::endl;
  return row;
}

size_t SensorParams::colFromAngle(const Angle& angle) const 
{
  size_t col = findClosest(_col_angles, angle);
  //handle CW or CCW direction
  if (_scan_direction == ScanDirection::CLOCK_WISE) {
    col = (_col_angles.size() - 1) - col; //flip the columns
  }
  return col;
}

size_t SensorParams::findClosest(const std::vector<Angle>& vec, const Angle& val) 
{
  size_t found = 0;
  if (vec.front() < vec.back()) {
    found = std::upper_bound(vec.begin(), vec.end(), val) - vec.begin();
  } else {
    found = vec.rend() - std::upper_bound(vec.rbegin(), vec.rend(), val);
  }
  if (found == 0) {
    return found;
  }
  if (found == vec.size()) {
    return found - 1;
  }
  auto diff_next = Angle::abs(vec[found] - val);
  auto diff_prev = Angle::abs(val - vec[found - 1]);
  return diff_next < diff_prev ? found : found - 1;
}

std::unique_ptr<SensorParams> SensorParams::VLP_16() 
{
  std::vector<float> correction_table = {15,13,11,9,7,5,3,1,-1,-3,-5,-7,-9,-11,-13,-15};
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(15_deg, -15_deg, 16, correction_table),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::VLP_32() 
{
  std::vector<float> correction_table = {15,10.333,7,4.667,3.333,2.333,1.667,1.333,1,0.667,0.333,0,-0.333,-0.667,-1,-1.333,
                                         -1.667,-2,-2.333,-2.667,-3,-3.333,-3.667,-4,-4.667,-5.333,-6.148,-7.254,-8.843,-11.31,-15.639,-25};
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(15_deg, -25_deg, 32, correction_table),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_32() 
{
  std::vector<float> correction_table = {10.67,9.33,8,6.67,5.33,4,2.67,1.33,0,-1.33,-2.67,-4,-5.33,-6.67,-8,-9.33,
                                        -10.67,-12,-13.33,-14.67,-16,-17.33,-18.67,-20,-21.33,-22.67,-24,-25.33,-26.67,-28,-29.33,-30.67};
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(10.0_deg, -30.0_deg, 32, correction_table),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_64_EQUAL() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(2.0_deg, -24.8_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_64_S2() 
{
  std::vector<float> correction_table = {4.97009,4.49316,4.00396,3.5025,2.97714,2.48635,1.9718,1.44522,
                                         0.976948,0.508544,-0.217594,-0.568941,-1.15441,-1.58751,-2.05552,-2.59339,
                                         -3.18918,-3.71431,-4.16892,-4.70446,-5.1927,-5.6686,-6.25946,-6.86054,
                                         -7.26426,-7.78227,-8.35633,-8.76862,-9.07171,-9.33972,-9.61906,-9.81801,
                                         -9.99435,-10.3629,-10.5387,-10.8608,-10.9457,-11.5203,-12.0702,-12.417,
                                         -12.9743,-13.4073,-14.0814,-14.5981,-15.1778,-15.6893,-16.1118,-16.554,
                                         -17.112,-17.7622,-18.2178,-18.7236,-19.1845,-19.5702,-20.1194,-20.8593,
                                         -21.308,-21.8851,-22.3575,-22.7272,-23.184,-23.8536,-24.4193,-24.8451};
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(2.0_deg, -24.8_deg, 64, correction_table),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_64_S3() 
{
  std::vector<float> correction_table = {2.28525,1.9601,1.64184,1.30008,0.986315,0.630338,0.283682,-0.088762,
                                        -0.414352,-0.772704,-1.02094,-1.44004,-1.76304,-2.10698,-2.42973,-2.8014,
                                        -3.1378,-3.49733,-3.81227,-4.19222,-4.46007,-4.82077,-5.19967,-5.58044,
                                        -5.86106,-6.20854,-6.51856,-6.87439,-7.20202,-7.57522,-7.84665,-8.28753,
                                        -8.61201,-9.19273,-9.62589,-9.93754,-10.5658,-11.1893,-11.6215,-12.1553,
                                        -12.5777,-12.9098,-13.425,-14.2421,-14.7515,-15.2585,-15.7088,-16.0361,
                                        -16.7432,-17.2659,-17.8026,-18.3023,-18.6984,-19.0674,-19.7633,-20.3359,
                                        -20.8757,-21.4035,-21.8508,-22.3214,-22.8168,-23.3184,-23.9378,-24.3474};
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(2.0_deg, -24.0_deg, 64, correction_table),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_64_GENERAL() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  AngularRange span_top(2.0_deg, -8.5_deg, 32);
  AngularRange span_bottom(-8.87_deg, -24.87_deg, 32);
  std::vector<AngularRange> spans = {{span_top, span_bottom}};
  params.setSpan(spans, AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_16_0512() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 512),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_16_1024() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 1024),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_16_2048() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 2048),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_64_0512() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 512),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_64_1024() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 1024),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_64_2048() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 2048),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::fullSphere(const Angle& discretization) 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, discretization),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(-90_deg, 90_deg, discretization),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

void SensorParams::fillCosSin() 
{
  _row_angles_sines.clear();
  _row_angles_cosines.clear();
  for (const auto& angle : _row_angles) {
    _row_angles_sines.push_back(sin(angle.val()));
    _row_angles_cosines.push_back(cos(angle.val()));
  }
  _col_angles_sines.clear();
  _col_angles_cosines.clear();
  for (const auto& angle : _col_angles) {
    _col_angles_sines.push_back(sin(angle.val()));
    _col_angles_cosines.push_back(cos(angle.val()));
  }
}



}  // namespace cloud_to_image
