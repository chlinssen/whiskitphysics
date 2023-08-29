/*
WHISKiT Physics Simulator
Copyright (C) 2019 Nadina Zweifel (SeNSE Lab)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string.h>
#include <yaml-cpp/yaml.h>

using namespace std;


typedef YAML::Node Parameters;

const YAML::Node mergeNodes(const YAML::Node &defaultNode, const YAML::Node &overrideNode);

Parameters read_parameters_from_file(string fn);

#endif
