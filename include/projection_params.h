
#ifndef PROJECTION_PARAMS_H_
#define PROJECTION_PARAMS_H_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include "sensor_params.h"

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
 * @brief Class for projection parameters.
 */
class ProjectionParams 
{
  public:
    using Ptr = shared_ptr<ProjectionParams>;
    using ConstPtr = const shared_ptr<const ProjectionParams>;

    ProjectionParams() {}
    ~ProjectionParams() {}

    /**
     * @brief      Parameters from the specified sensor model
     * @param[in]  model the sensor model to query
     * @return     A pointer to parameters
     */
    std::unique_ptr<SensorParams> fromModel(const std::string& model);

    const SensorParams& fromModel(const std::string& model) const;

    std::unique_ptr<SensorParams> operator [](const std::string& model) { return fromModel(model); }

    const SensorParams& operator [](const std::string& model) const { return fromModel(model); } 

    bool sensorExists(const std::string& model) const { return (_projection_params.size() && _projection_params.find(model) != _projection_params.end()); };

    friend YAML::Emitter& operator << (YAML::Emitter& out, ProjectionParams& val)
    {
      //skip saving an empty map
      if (!val._projection_params.size()) {
        return out;
      }
      out << YAML::Comment("LiDARs projection parameters");   
      out << YAML::BeginSeq;
      for (auto& it : val._projection_params) {
        out << YAML::BeginMap;
        out << YAML::Key << "name";
        out << YAML::Value << it.first;
        out << YAML::Key << "params";
        out << YAML::Value << *(it.second);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;

      return out;
    }

    friend std::ostream& operator << (std::ostream& out, ProjectionParams& val)
    {
      //skip saving an empty map
      if (!val._projection_params.size()) {
        return out;
      }
      out << "#LiDARs projection parameters" << std::endl;
      for (auto& it : val._projection_params) {
        out << "name: ";
        out << it.first << std::endl;
        out << "params: " << std::endl;
        out << *(it.second);
      }

      return out;
    }

    void loadFromFile(const std::string& filename);
    void saveToFile(const std::string& filename);

    void genSampleSensors();

    bool valid();

  private:
    std::map<std::string, std::unique_ptr<SensorParams> > _projection_params;
};

}  // namespace cloud_to_image

#endif  // PROJECTION_PARAMS_H
