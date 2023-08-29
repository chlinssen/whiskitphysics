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

This code is based on code published by
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

*/

#include "Simulation.h"
#include "Parameters.h"
#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Quaternion.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"

#include <iostream>
#include <boost/program_options.hpp>

#include <signal.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#include <boost/lexical_cast.hpp>

using boost::lexical_cast;

volatile sig_atomic_t exitFlag = 0;

void exit_function(int sigint)
{
	exitFlag = 1;
}
template <typename Out>
void split(const std::string &s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int main(int argc, char* argv[])
{
	signal(SIGINT, exit_function);

	std::cout.precision(17);

	std::string parameters_fn;

  	try
  	{ /** Define and parse the program options  */
		namespace po = boost::program_options;
		po::options_description desc("Options");
		desc.add_options()
		("help,h", "Help screen")
		("parameters", po::value<std::string>(&parameters_fn), "Filename or command-separated list of filenames, containing default parameters in YAML format");

		po::variables_map vm;

		try {
			po::store(po::parse_command_line(argc, argv, desc,po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm); // can throw
		 	po::notify(vm);

		 	if ( vm.count("help")  ) {
				std::cout << "Bullet Whisker Simulation" << std::endl
						  << desc << std::endl;
				return 0;
			}

			Parameters parameters;
			for (std::string fn : split(parameters_fn, ',')) {
  				parameters = mergeNodes(parameters, read_parameters_from_file(fn));
			}

		  	DummyGUIHelper noGfx;
			CommonExampleOptions options(&noGfx);
			Simulation* simulation = new Simulation(options.m_guiHelper);
			simulation->load_parameters(parameters);
			simulation->initPhysics();

			// run simulation
			do {
				simulation->stepSimulation();
			} while (!(exitFlag || simulation->exitSim) );

			std::cout << "Saving data..." << std::endl;
			if (parameters["SAVE"].as< bool >()) {
				std::cout << "Simualtion terminated." << std::endl;
				output* results = simulation->get_results();
				const string output_dir = parameters["dir_out"].as< string >();
				save_data(results, output_dir);
			}

			std::cout << "Exit simulation..." << std::endl;
			simulation->exitPhysics();

			delete simulation;
			std::cout << "Done." << std::endl;

		}
		catch(po::error& e)
		{
		  std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		  std::cerr << desc << std::endl;
		  return 1;
		}
  	}
  	catch(std::exception& e)
  	{
		std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
		return 2;
  	}

	return 0;
}
