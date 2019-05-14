#include "projection_params.h"
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>

using namespace cloud_to_image;

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	//test projection parameters creation
	ProjectionParams proj_params;
	proj_params.genSampleSensors();
	std::cout << proj_params << std::endl;

	//test projection parameters saving
	std::string filename = std::string("/tmp/test_projection_params.yaml");
	proj_params.saveToFile(filename);

	//test projection parameters loading
	proj_params.loadFromFile(filename);
	std::cout << "projection params from file \"" << filename << "\":" << std::endl;
	std::cout << proj_params << std::endl;

	//test access to sensor parameters
	std::cout << "VLP16 projection params:" << std::endl;
	std::cout << *(proj_params.fromModel("VLP-16")) << std::endl;
	std::cout << "HDL32 projection params:" << std::endl;
	std::cout << *(proj_params.fromModel("HDL-32")) << std::endl;
	std::cout << "HDL64 projection params:" << std::endl;
	std::cout << *(proj_params["HDL-64"]) << std::endl;

	//test YAML output	
	YAML::Emitter out;
	out << YAML::BeginMap;
	out << YAML::Key << "ProjParams";
	out << YAML::Value << proj_params;
	out << YAML::EndMap;
	std::cout << "projection params:" << std::endl;
	std::cout << out.c_str() << std::endl;
	
	return 0;
}