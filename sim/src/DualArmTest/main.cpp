#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include "../../include/MR/modern_robotics.h"
#include "../../include/MR/MR_Indy7.h"
#include "../../include/MR/MR_Indy7_DualArm.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include <Eigen/Dense>
#include "Utils/b3Clock.h"
#include "Indy7.h"

#include "../../include/MR/modern_robotics.h"
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
	double t = 0;
	JVec MAX_TORQUES;
	MAX_TORQUES<<1000,1000,1000,1000,1000,1000;
	JVec HOME_LEFT,HOME_RIGHT;
	HOME_LEFT<< -0.1455808405866516 , -0.8133711809728681  ,-1.3075236414349891 , -0.6146456270149877  ,-1.0306524888727113  ,0.06930112107483291 ;
	HOME_RIGHT<< 0.1455808405866516 , 0.8133711809728681  ,1.3075236414349891 , 0.6146456270149877  ,1.0306524888727113 ,-0.06930112107483291 ;
	//HOME_LEFT<< -0.166785,-0.112846,-2.803911,0.061847,-0.224256,0.5;
	//HOME_RIGHT<< 0.147439,0.258307,-2.698818,0.281465,-0.200132,-0.849611;
	left_arm.resetQ(&sim,HOME_LEFT);
	right_arm.resetQ(&sim,HOME_RIGHT);
	dualarm.left_arm->q = left_arm.getQ( &sim);
	dualarm.right_arm->q = right_arm.getQ( &sim);
	while(1){
		dualarm.left_arm->q = left_arm.getQ( &sim);
		dualarm.right_arm->q = right_arm.getQ( &sim);
		JVec gravLeft = dualarm.left_arm->Gravity( dualarm.left_arm->q);
		JVec gravRight = dualarm.right_arm->Gravity( dualarm.right_arm->q);

 		left_arm.setTorques(&sim,   gravLeft , MAX_TORQUES);
		right_arm.setTorques(&sim,  gravRight , MAX_TORQUES);
 		sim.stepSimulation();
 		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
 		t = t+FIXED_TIMESTEP;	
 	}		

    
}
