#include "angles.h"
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>

using namespace cloud_to_image;

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	//test angular range creation
	Angle start_angle = -180_deg;
	Angle end_angle = 180_deg;
	Angle step = 0.1_deg;
	auto range = AngularRange(start_angle, end_angle, step);

	std::cout << range << std::endl;

	//test YAML output
	YAML::Emitter out;
	out << YAML::BeginMap;
	out << YAML::Key << "angle_range";
	out << YAML::Value << range;
	out << YAML::EndMap;
	std::cout << out.c_str() << std::endl;

	//test YAML file creation
	std::string filename = std::string("/tmp/test_angular_range.yaml");
	std::ofstream ofs(filename, std::ofstream::out);
  	ofs << out.c_str();
  	ofs.close();

  	//test YAML loading
  	YAML::Node config = YAML::LoadFile(filename);
  	AngularRange range2 = config["angle_range"].as<AngularRange>();

	std::cout << range2 << std::endl;

	return 0;
}