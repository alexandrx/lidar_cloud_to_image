#include "angles.h"
#include "sensor_params.h"
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>

using namespace cloud_to_image;

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	//test sensor params creation
	auto params = SensorParams();
  	params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                   AngularRange::Direction::HORIZONTAL);
  	params.setSpan(AngularRange(15_deg, -15_deg, 16),
                   AngularRange::Direction::VERTICAL);
  	if (!params.valid()) {
    	fprintf(stderr, "ERROR: params are not valid!\n");
    	return 1;
  	}

  	std::cout << "VLP16 sensor params:" << std::endl;
	std::cout << params << std::endl;

	//test YAML output
	YAML::Emitter out;
	out << YAML::BeginMap;
	out << YAML::Key << "VLP16";
	out << YAML::Value << params;
	out << YAML::EndMap;
	std::cout << out.c_str() << std::endl;

	//test YAML file creation
	std::string filename = std::string("/tmp/test_sensor_params.yaml");
	std::ofstream ofs(filename, std::ofstream::out);
  	ofs << out.c_str();
  	ofs.close();

  	//test YAML loading
  	YAML::Node config = YAML::LoadFile(filename);
  	SensorParams params2 = config["VLP16"].as<SensorParams>();

  	std::cout << "VLP16 sensor params from file \"" << filename << "\":" << std::endl;
	std::cout << params2 << std::endl;

	//test sensor params generators 
	auto params3 = SensorParams::VLP_16();
  	std::cout << "VLP16 sensor params from generator" << std::endl;
	std::cout << *params3 << std::endl;

	return 0;
}