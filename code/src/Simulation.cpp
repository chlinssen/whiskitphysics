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

btVector4 BLUE = btVector4(0.,0.,1.0,1);
btVector4 RED = btVector4(1.,0.,0.0,1);
btVector4 GREEN = btVector4(0.,1.,0.0,1);
btVector4 GREY = btVector4(0.,0.,0.0,0.5);
btVector4 YELLOW = btVector4(1.,1.,0.0,1);
btVector4 ORANGE = btVector4(1.,0.647,0.0,1);

void Simulation::load_parameters(Parameters& parameters) {
	TIME_STEP = parameters["TIME_STEP"].as< float >();
	NUM_STEP_INT = parameters["NUM_STEP_INT"].as< int >();
	TIME_STOP = parameters["TIME_STOP"].as< float >();
	NO_WHISKERS = parameters["NO_WHISKERS"].as< bool >();
	SAVE = parameters["SAVE"].as< bool >();
	ACTIVE = parameters["ACTIVE"].as< bool >();
	EXPLORING = parameters["EXPLORING"].as< bool >();
	DEBUG = parameters["DEBUG"].as< bool >();
	PRINT = parameters["PRINT"].as< int >();


	camPos = parameters["CPOS"].as< std::vector< float > >();
	camDist = parameters["CDIST"].as< float >();
	camPitch = parameters["CPITCH"].as< float >();
	camYaw = parameters["CYAW"].as< float >();

	file_whisking_angle = parameters["file_whisking_angle"].as< std::string >();
	WHISKER_PARAMS_DIR = parameters["WHISKER_PARAMS_DIR"].as< std::string >();
	dir_rathead_trajectory = parameters["dir_rathead_trajectory"].as< std::string >();

	OBJECT = parameters["OBJECT"].as< int >();
	PEG_LOC = parameters["PEG_LOC"].as< std::vector< float > >();
	PEG_SPEED = parameters["PEG_SPEED"].as< float >();
	file_env = parameters["file_env"].as< std::string >();
	std::cout <<"aefwefwef4\n";

	// create rat
	rat = new Rat(m_guiHelper,m_dynamicsWorld, &m_collisionShapes, parameters);
	btVector3 rathead_pos = rat->getPosition();

	vec = btVector3(0.5,-1,0).normalized();
	const std::vector < std::string > whisker_names = parameters["WHISKER_NAMES"].as< std::vector< std::string > >();

	std::cout << "Whiskers to simulate: ";
	for (std::string s : whisker_names) {
		std::cout << "s ";
	}
	std::cout << std::endl;

	data_dump->init(whisker_names);

	// set camera position to rat head
	camPos[0] = rathead_pos[0]+camPos[0];
	camPos[1] = rathead_pos[1]+camPos[1];
	camPos[2] = rathead_pos[2]+camPos[2];
	std::cout <<"aefwefwe5\n";

}

void Simulation::stepSimulation(){
std::cout << "simulation->stepSimulation();...\n";

	auto start = std::chrono::high_resolution_clock::now();
	m_time += TIME_STEP; 								// increase time
	m_step += 1;													// increase step

	// run simulation as long as stop time not exceeded
	if(TIME_STOP == 0 || m_time < TIME_STOP) {

		// register collisions
		rat->detect_collision(m_dynamicsWorld);
std::cout << "simulation->stepSimulation() 1...\n";

		// first, push back data into data_dump
		if(!NO_WHISKERS && SAVE) {
			rat->dump_M(data_dump);
			rat->dump_F(data_dump);
			rat->dump_Q(data_dump);
		}
std::cout << "simulation->stepSimulation()2...\n";

		// moving object 1
		if(OBJECT==1){
			if(PEG_SPEED > 0){
std::cout << "simulation->stepSimulation() 2a...\n";
				btVector3 velocity = PEG_SPEED * btVector3(0.4,-1,0).normalized();
std::cout << "simulation->stepSimulation() 2b...\n";
std::cout << "\tpeg = " << peg << "\n";
				peg->setLinearVelocity(velocity);
			}
std::cout << "simulation->stepSimulation()3...\n";
		}
std::cout << "simulation->stepSimulation()3a...\n";
std::cout << "\trat = " << rat << "\n";

		// move array if in ACTIVE mode
		if(ACTIVE && !NO_WHISKERS){
			rat->whisk(m_step, whisker_vel);
		}

std::cout << "simulation->stepSimulation()4...\n";

		// move rat head if in EXPLORING mode
		if(EXPLORING){
			this_loc_vel = HEAD_LOC_VEL[m_step-1];
			rat->setLinearVelocity(btVector3(this_loc_vel[3], this_loc_vel[4], this_loc_vel[5]/10));
			// rat->setLinearVelocity(btVector3(0, 0, 0));
			rat->setAngularVelocity(btVector3(this_loc_vel[6], this_loc_vel[7], this_loc_vel[8]));
			// rat->setAngularVelocity(btVector3(0, 0, 0));
		}
std::cout << "simulation->stepSimulation()5...\n";

		// step simulation
		m_dynamicsWorld->stepSimulation(TIME_STEP,
		                                NUM_STEP_INT,
										TIME_STEP / NUM_STEP_INT);
std::cout << "simulation->stepSimulation()6...\n";

		// draw debug if enabled
	    if(DEBUG) {
	    	m_dynamicsWorld->debugDrawWorld();
	    }
std::cout << "simulation->stepSimulation()7...\n";

	    // set exit flag to zero
	    exitSim = 0;
	}
	else{
		// timeout -> set exit flg
		exitSim = 1;
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	m_time_elapsed += duration.count()/1000.f;
	auto factor = m_time_elapsed / m_time;
	auto time_remaining = (int)((TIME_STOP - m_time) * (factor));
	if(PRINT==2){
		std::cout << "\rSimulation time: " << std::setprecision(2) << m_time << "s\tCompleted: " << std::setprecision(2) << m_time/TIME_STOP*100 << " %\tTime remaining: " << std::setprecision(4) << time_remaining/60 << " min " << std::setprecision(4) << (time_remaining % 60) << " s\n" << std::flush;
	}

}

void Simulation::initPhysics()
{


	// set visual axis
	m_guiHelper->setUpAxis(2);
	std::cout <<"aefwefwe9999 a\n";

	// create empty dynamics world[0]
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    // broadphase algorithm
    m_broadphase = new btDbvtBroadphase();

	// select solver
	std::cout << "Using btSequentialImpulseConstraintSolver..." << std::endl;
	m_solver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	// set number of iterations
	m_dynamicsWorld->getSolverInfo().m_numIterations = 20;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD |
                        SOLVER_USE_WARMSTARTING |
                        SOLVER_RANDMIZE_ORDER |
                        0;
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	m_dynamicsWorld->getSolverInfo().m_erp = 0.8f;

	std::cout <<"aefwefwe9999 b\n";

	if (rat) {
		rat->initPhysics(m_dynamicsWorld);
	}

	std::cout <<"aefwefwe12312312312354\n";

	// set gravity
	m_dynamicsWorld->setGravity(btVector3(0,0,0));

    // create debug drawer
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer()){
		if(DEBUG==1){
			std::cout << "DEBUG option 1: wireframes." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		}
		else if(DEBUG==2){
			std::cout << "DEBUG option 2: constraints." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints);
		}
		else if(DEBUG==3){
			std::cout << "DEBUG option 3: wireframes & constraints." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawConstraintLimits);
		}
		else if(DEBUG==4){
			std::cout << "DEBUG option 4: AAbb." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawAabb);
		}
		else if(DEBUG==5){
			std::cout << "DEBUG option 5: Frammes." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawFrames);
		}
		else if(DEBUG==6){
			std::cout << "DEBUG option 6: Only collision" << std::endl;
			// m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawFrames);
		}
		else{
			std::cout << "No DEBUG." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);
		}
	}
	std::cout <<"aefwefwe9999 c\n";

	// create object to collide with
	// peg
	if(OBJECT==1){
		btCollisionShape* pegShape = new btCylinderShapeZ(btVector3(1,1,80));
		pegShape->setMargin(0.1);
		m_collisionShapes.push_back(pegShape);
		btTransform trans = createFrame(btVector3(PEG_LOC[0], PEG_LOC[1], PEG_LOC[2]),btVector3(0, 0, 0));
		peg = createDynamicBody(1,0.5,trans, pegShape, m_guiHelper,  BLUE);
		m_dynamicsWorld->addRigidBody(peg,COL_ENV,envCollidesWith);
		peg->setActivationState(DISABLE_DEACTIVATION);

	}
	// create object to collide with wall
	else if(OBJECT==2){
		btCollisionShape* wallShape = new btBoxShape(btVector3(5,200,60));
		wallShape->setMargin(0.1);
		m_collisionShapes.push_back(wallShape);
		btTransform trans = createFrame(btVector3(50,0,0),btVector3(0,0,PI/6));
		wall = createDynamicBody(0,0.5, trans, wallShape, m_guiHelper,  BLUE);
		m_dynamicsWorld->addRigidBody(wall,COL_ENV,envCollidesWith);
	}
	// create object from 3D scan
	else if(OBJECT==3){
		// add environment to world
		btVector4 envColor = btVector4(0.6,0.6,0.6,1);
		env = new Object(m_guiHelper,m_dynamicsWorld, &m_collisionShapes,btTransform(),file_env,envColor,btScalar(SCALE),btScalar(0),COL_ENV,envCollidesWith);
	}
	std::cout <<"aefwefwe9999 d\n";

	// generate graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	resetCamera();

	// if active whisking, load whisking protraction angle trajectory
	if (ACTIVE) {
		const std::string dir_param = WHISKER_PARAMS_DIR;

		read_csv_float(dir_param + file_whisking_angle, whisker_vel);
		TIME_STOP = std::min(TIME_STOP, (whisker_vel[0].size()/3 - 1) * TIME_STEP);
	}

	// if exploring, load data for rat head trajectory
	if (EXPLORING){
		read_csv_float(dir_rathead_trajectory, HEAD_LOC_VEL);
	}
	std::cout <<"aefwefwe9999 e\n";

	// initialize time/step tracker
	m_time_elapsed = 0;
	m_time = 0;
	m_step = 0;

	std::cout << "\n\nStart simulation..." << std::endl;
	std::cout << "\n====================================================\n" << std::endl;
}

output* Simulation::get_results(){
	return data_dump;
}

void Simulation::resetCamera(){
	m_guiHelper->resetCamera(camDist,camYaw,camPitch,camPos[0],camPos[1],camPos[2]);
}
