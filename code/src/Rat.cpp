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

#include "Rat.h"

Rat::Rat(GUIHelperInterface* helper,btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameters& parameters){
	const std::vector< float > RATHEAD_LOC = parameters["RATHEAD_LOC"].as< std::vector< float > >();
	const std::vector< float > RATHEAD_ORIENT = parameters["RATHEAD_ORIENT"].as< std::vector< float > >();
	const std::string dir_rathead = parameters["dir_rathead"].as< std::string >();

	std::vector<std::string> whisker_names = parameters["WHISKER_NAMES"].as< std::vector< std::string > >();

	if (whisker_names[0] == "ALL") {
		whisker_names = {
			"LA0","LA1","LA2","LA3","LA4",
			"LB0","LB1","LB2","LB3","LB4",
			"LC0","LC1","LC2","LC3","LC4","LC5",
			"LD0","LD1","LD2","LD3","LD4","LD5",
			"LE1","LE2","LE3","LE4","LE5",
			"RA0","RA1","RA2","RA3","RA4",
			"RB0","RB1","RB2","RB3","RB4",
			"RC0","RC1","RC2","RC3","RC4","RC5",
			"RD0","RD1","RD2","RD3","RD4","RD5",
			"RE1","RE2","RE3","RE4","RE5"};
	}
	else if (whisker_names[0] == "R") {
		whisker_names = {

			"RA0","RA1","RA2","RA3","RA4",
			"RB0","RB1","RB2","RB3","RB4",
			"RC0","RC1","RC2","RC3","RC4","RC5",
			"RD0","RD1","RD2","RD3","RD4","RD5",
			"RE1","RE2","RE3","RE4","RE5"};
	}
	else if (whisker_names[0] == "L") {
		whisker_names = {
			"LA0","LA1","LA2","LA3","LA4",
			"LB0","LB1","LB2","LB3","LB4",
			"LC0","LC1","LC2","LC3","LC4","LC5",
			"LD0","LD1","LD2","LD3","LD4","LD5",
			"LE1","LE2","LE3","LE4","LE5"};
	}

	std::cout <<"aefwefwe11111 c\n";

	// set initial position and orientation of rat head
	btVector3 position = btVector3(RATHEAD_LOC[0], RATHEAD_LOC[1], RATHEAD_LOC[2]);
	btVector3 orientation = btVector3(RATHEAD_ORIENT[0], RATHEAD_ORIENT[1], RATHEAD_ORIENT[2]);
	std::cout <<"aefwefwe11111 ccc\n";

	// create transform for ratHead
	btTransform headTransform = createFrame(position, orientation);
	std::cout <<"aefwefwe11111 cc\n";

	// define shape and body of head (mass=100)
	btVector4 color = btVector4(0.1,0.1,0.1,1);
	rathead = new Object(helper,world,shapes,headTransform,dir_rathead,color,SCALE/10,100.,COL_HEAD,headCollidesWith);
	std::cout <<"aefwefwe11111 aadadada\n";

	std::cout <<"aefwefwe11111 d\n";

	// create new Whiskers for this rat head
	// origin: mean position of all basepoints
	btTransform head2origin = createFrame(originOffset,originOrientation);

	// create Whiskers
	if(!parameters["NO_WHISKERS"].as< bool >()) {
		for(int w=0;w<whisker_names.size();w++){
			std::cout << "\tcreating whisker " << w << "\n";
			Whisker* whisker = new Whisker(helper, shapes, whisker_names[w], parameters, head2origin);
			m_whiskerArray.push_back(whisker);
		}
	}
std::cout << "\twhisker_names = " << whisker_names.size()<< "\n";
	std::cout <<"finished rat construction\n";
}

void Rat::initPhysics(btDiscreteDynamicsWorld* world) {
	std::cout<<" Rat::initPhysics\n";
	// set rathead->body to active state
	rathead->initPhysics(world);
	rathead->body->setActivationState(DISABLE_DEACTIVATION);
	for(int w = 0; w < m_whiskerArray.size(); ++w) {
		std::cout << "whisker " << w << " initphys\n";
		m_whiskerArray[w]->initPhysics(world, rathead->body);
	}
}

btAlignedObjectArray<Whisker*> Rat::getArray(){
	return m_whiskerArray;
}

Whisker* Rat::getWhisker(int index){
	return m_whiskerArray[index];
}

const btVector3 Rat::getPosition(){
	return rathead->body->getCenterOfMassPosition();
}

const btTransform Rat::getTransform(){
	return rathead->body->getCenterOfMassTransform();
}

void Rat::setTransform(btTransform tr){
	rathead->body->setCenterOfMassTransform(tr);
}

void Rat::setLinearVelocity(btVector3 shift){
	rathead->body->setLinearVelocity(shift);
}
void Rat::setAngularVelocity(btVector3 rotation){
	rathead->body->setAngularVelocity(rotation);
}
const btVector3 Rat::getLinearVelocity(){
	return rathead->body->getLinearVelocity();
}
const btVector3 Rat::getAngularVelocity(){
	return rathead->body->getAngularVelocity();
}

void Rat::whisk(int step, std::vector<std::vector<float>> whisker_vel){
std::cout << "Rat::whisk\n";

	// total number of steps in one cycle of whisking phase
	int totalStep = whisker_vel[0].size()/3;

	// for every whisker, read its angular velocity at this step
	for (int i=0;i<m_whiskerArray.size();i++){
std::cout << "Rat::whisk " << i << "\n";
		int idx = m_whiskerArray[i]->idx;
std::cout << "Rat::whisk " << idx << "\n";
std::cout << "Rat::whisk totalstep = " << totalStep << "\n";
		btScalar a_vel_0 = whisker_vel[idx][(step%totalStep)*3-3];
		btScalar a_vel_1 = whisker_vel[idx][(step%totalStep)*3-2];
		btScalar a_vel_2 = whisker_vel[idx][(step%totalStep)*3-1];
		m_whiskerArray[i]->whisk(a_vel_0, a_vel_1, a_vel_2, getAngularVelocity());
	}
std::cout << "end Rat::whisk\n";
}

// function to retrieve torques at base points
void Rat::dump_M(output* data){

	std::vector<btScalar> mx;
	std::vector<btScalar> my;
	std::vector<btScalar> mz;

	for(int w=0; w < m_whiskerArray.size(); w++){
		btVector3 moments = m_whiskerArray[w]->getTorques();
		mx.push_back(moments[0]);
		my.push_back(moments[1]);
		mz.push_back(moments[2]);
	}
	data->Mx.push_back(mx);
	data->My.push_back(my);
	data->Mz.push_back(mz);
}

// function to retrieve forces at base points
void Rat::dump_F(output* data){

	std::vector<btScalar> fx;
	std::vector<btScalar> fy;
	std::vector<btScalar> fz;

	for(int w=0; w < m_whiskerArray.size(); w++){
		btVector3 forces = m_whiskerArray[w]->getForces();
		fx.push_back(forces[0]);
		fy.push_back(forces[1]);
		fz.push_back(forces[2]);
	}
	data->Fx.push_back(fx);
	data->Fy.push_back(fy);
	data->Fz.push_back(fz);
}

// function to obtain x coordinates of all whisker units
void Rat::dump_Q(output* data){

	for(int w=0; w < m_whiskerArray.size(); w++){
		data->Q[w].X.push_back(m_whiskerArray[w]->getX());
		data->Q[w].Y.push_back(m_whiskerArray[w]->getY());
		data->Q[w].Z.push_back(m_whiskerArray[w]->getZ());
		data->Q[w].C.push_back(m_whiskerArray[w]->getCollision());
	}
}

// detect collisions between whiskers and objects
void Rat::detect_collision(btDiscreteDynamicsWorld* world){

// Code adapted from: https://andysomogyi.github.io/mechanica/bullet.html

	world->performDiscreteCollisionDetection();

	int numManifolds = world->getDispatcher()->getNumManifolds();
	//For each contact manifold
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = const_cast<btCollisionObject*>(contactManifold->getBody1());
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
		int numContacts = contactManifold->getNumContacts();

		//For each contact point in that manifold
		for (int j = 0; j < numContacts; j++) {
			//Get the contact information
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();
			double ptdist = pt.getDistance();


			if (ptdist < 0.5){
				int* coll0 = (int*) obA->getUserPointer();
				if(coll0!=nullptr){
					*coll0 = 1;
				}
				int* coll1 = (int*) obB->getUserPointer();
				if(coll1!=nullptr){
					*coll1 = 1;
				}
			}

		}
	}
}
