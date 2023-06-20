#include <Eigen/Dense>
#include <iostream>
#include <chrono>

#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include <Eigen/Dense>
#include "Utils/b3Clock.h"
#include "Indy7.h"

#include "modern_robotics.h"
#include "modern_robotics_relative.h"
#include "MR_Indy7.h"
#include "MR_Indy7_DualArm.h"


#include <vector>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace mr;


extern const int CONTROL_RATE;
const int CONTROL_RATE = 1000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;
MR_Indy7_DualArm dualarm;

// relmr::JVec setq(mr::JVec q_l,mr::JVec q_r ){
// 		relmr::JVec q = relmr::JVec::Zero();
// 		q.segment<6>(0) = q_l;
// 		q.segment<6>(6) = -q_r.reverse();
// 		return q;
// }

int main()
{
	dualarm=MR_Indy7_DualArm();
	dualarm.MRSetup();
	cout<<"START PROGRAM"<<endl;

	b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	if (!b3CanSubmitCommand(client))
	{
	printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
	exit(0);
	}
	b3RobotSimulatorClientAPI_InternalData data;
	data.m_physicsClientHandle = client;
	data.m_guiHelper = 0;
	b3RobotSimulatorClientAPI_NoDirect sim;
	sim.setInternalData(&data);


	sim.resetSimulation();
	sim.setGravity( btVector3(0 , 0 ,-9.8));
    int bodyId = sim.loadURDF("model/body.urdf");  	
	int leftId = sim.loadURDF("model/indy7.urdf");  
	int rightId = sim.loadURDF("model/indy7.urdf");  
	Indy7 left_arm(&sim,leftId);
	Indy7 right_arm(&sim,rightId);
	btVector3 left_pos(0,0.1563,0.3772);
	btQuaternion left_orn(-0.5,0,0,0.866);
	btVector3 right_pos(0,-0.1563,0.3772);
	btQuaternion right_orn(0.0,0.5,-0.866,0);
	sim.resetBasePositionAndOrientation(leftId,left_pos, left_orn);
	sim.resetBasePositionAndOrientation(rightId,right_pos, right_orn);
	double t= 0;
	while(1){

 		sim.stepSimulation();
 		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
 		t = t+FIXED_TIMESTEP;	
 	}		

    
}

