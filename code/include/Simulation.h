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

#ifndef SIMULATION_H
#define SIMULATION_H

#include "Rat.h"
#include "Object.h"
#include "Simulation_utility.h"
#include "Simulation_IO.h"

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> // Add this include for inet_pton
#include <chrono>
#include <iomanip>      // std::setprecision

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btQuaternion.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "CommonInterfaces/CommonParameterInterface.h"

#include "../jsonlib/json.hpp"

using json = nlohmann::json;


class Simulation* SimulationCreateFunc(struct CommonExampleOptions& options);

class Simulation : public CommonRigidBodyBase
{
public:
	Simulation(struct GUIHelperInterface* helper):CommonRigidBodyBase(helper){}
	virtual ~Simulation(){}
	virtual void initPhysics();
	void load_parameters(Parameters& parameters);

	virtual void stepSimulation();

	output* get_results();
	void resetCamera();

	bool exitSim;

    int init_socket();
private:
    const int SOCKET_PORT = 12346;
    int clientSocket;
    struct sockaddr_in serverAddr;

	btScalar m_time_elapsed;
	btScalar m_time;
	int m_step;
	int m_total_steps;

	btAlignedObjectArray<btVector3> m_objcenter; // store center position calculated from bounding box for all objs, before start trans
    btAlignedObjectArray<btVector3> m_objboundingbox; // store bounding box for all objs, before start trans

	btRigidBody* peg;
	btRigidBody* wall;
	btVector3 vec;
	Rat* rat;
	Object* object;
	Object* env;
	output* data_dump = new output();
	std::vector< float > this_loc_vel;

	std::vector< std::vector< float > > whisker_vel;

	float TIME_STEP;
	int NUM_STEP_INT;
	int NUM_STEP_SOLVER;
	float TIME_STOP;
	bool NO_WHISKERS;
	bool SAVE;
	bool ACTIVE;
	bool EXPLORING;
	int PRINT;
	bool DEBUG;

	int OBJECT;
	std::vector< float > PEG_LOC;
	double PEG_SPEED;
	string file_env;

	std::vector< float > gravity;

	std::vector< std::vector< float > > HEAD_LOC_VEL;
	std::string file_whisking_angle;
	std::string dir_rathead_trajectory;
	std::string WHISKER_PARAMS_DIR;

	std::vector< float > camPos;
	btScalar camDist;
	btScalar camPitch;
	btScalar camYaw;
};

#endif //SIMULATION_H
