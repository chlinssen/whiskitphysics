#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string.h>
#include <yaml-cpp/yaml.h>

using namespace std;


typedef YAML::Node Parameters;

const YAML::Node mergeNodes(const YAML::Node &defaultNode, const YAML::Node &overrideNode);

Parameters read_parameters_from_file(string fn);


#endif
