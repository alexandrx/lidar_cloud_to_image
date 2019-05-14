
#include "projection_params.h"
#include "sensor_params.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

#include <yaml-cpp/yaml.h>

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

std::unique_ptr<SensorParams> ProjectionParams::fromModel(const std::string& model)
{
  //if list is empty or model unavailable
  if (!_projection_params.size() || _projection_params.find(model) == _projection_params.end()) {
    //return empty params
    auto params = SensorParams();
    return make_unique<SensorParams>(params);
  } else {
    return make_unique<SensorParams>(*(_projection_params.at(model)));
  }
}

const SensorParams& ProjectionParams::fromModel(const std::string& model) const 
{
  //if list is empty or model unavailable
  if (!_projection_params.size() || _projection_params.find(model) == _projection_params.end()) {
    //return empty params
    auto params = SensorParams();
    return *(make_unique<SensorParams>(params));
  } else {
    return *(_projection_params.at(model));
  }
}


void ProjectionParams::loadFromFile(const std::string& filename)
{
  if (_projection_params.size()) {
    _projection_params.clear();
  }

  YAML::Node config = YAML::LoadFile(filename);
  for(YAML::const_iterator iter=config.begin(); iter != config.end(); ++iter) {
    auto it = *iter;
    std::string model = it["name"].as<std::string>();
    auto params = it["params"].as<SensorParams>();

    // check validity
    if (!params.valid()) {
      fprintf(stderr, "ERROR: the config read was not valid.\n");
      exit(1);
    }
    _projection_params[model] = make_unique<SensorParams>(params);
  }
}

void ProjectionParams::saveToFile(const std::string& filename)
{
  //skip saving an empty map
  if (!_projection_params.size()) {
    return;
  }

  YAML::Emitter out;
  out << *this;

  std::ofstream ofs(filename, std::ofstream::out);
  ofs << out.c_str();
  ofs.close();
}

void ProjectionParams::genSampleSensors()
{
  if (_projection_params.size()) {
    _projection_params.clear();
  }

  //Velodyne
  _projection_params["VLP-16"] = SensorParams::VLP_16();
  _projection_params["VLP-32"] = SensorParams::VLP_32();
  _projection_params["HDL-32"] = SensorParams::HDL_32();
  _projection_params["HDL-64-S2"] = SensorParams::HDL_64_S2();
  _projection_params["HDL-64-S3"] = SensorParams::HDL_64_S3();
  //Ouster
  _projection_params["OS-1-16-0512"] = SensorParams::OS_1_16_0512();
  _projection_params["OS-1-16-1024"] = SensorParams::OS_1_16_1024();
  _projection_params["OS-1-16-2048"] = SensorParams::OS_1_16_2048();
  _projection_params["OS-1-64-0512"] = SensorParams::OS_1_64_0512();
  _projection_params["OS-1-64-1024"] = SensorParams::OS_1_64_1024();
  _projection_params["OS-1-64-2048"] = SensorParams::OS_1_64_2048();
}

bool ProjectionParams::valid()
{
  bool _valid = true;
  if (!_projection_params.size()) {
    return false;
  }
  for (auto& it : _projection_params) {
    _valid |= (it.first.size() && it.second->valid()); 
  }
  return _valid;
}

}  // namespace cloud_to_image
