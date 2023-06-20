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
#include "DualArm.h"

#include "modern_robotics.h"
#include "modern_robotics_relative.h"
#include "MR_Indy7.h"
#include "MR_Indy7_DualArm.h"
#include <matplot/matplot.h>


#include <vector>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace mr;
using namespace matplot;


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
	//-----MR SETUP START-------------------
	dualarm=MR_Indy7_DualArm();
	dualarm.MRSetup();
	//-----MR SETUP END-------------------
	cout<<"START PROGRAM"<<endl;


	//---------BULLET SETUP START------------------
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
	btVector3 left_pos(0,0.1563,0.3772);
	btQuaternion left_orn(-0.5,0,0,0.866);
	btVector3 right_pos(0,-0.1563,0.3772);
	btQuaternion right_orn(0.0,0.5,-0.866,0);    
	sim.resetBasePositionAndOrientation(leftId,left_pos, left_orn);
	sim.resetBasePositionAndOrientation(rightId,right_pos, right_orn);	
    DualArm robot=DualArm(&sim,rightId,leftId);
	//---------BULLET SETUP END-----------------
	double t= 0;
	std::vector<relmr::JVec> q_list;
	std::vector<double> t_list;
	double Tf = 5;
	double dt = FIXED_TIMESTEP;
	int N = int(Tf/dt);

	for(int i =0;i<N;i++){
		//JVec gravTau = mr::GravityForces(JVec::Zero(), Vector3d::Zero(), dualarm.left_arm->Mlist, dualarm.left_arm->Glist, dualarm.left_arm->Slist) ;
		relmr::JVec q = robot.get_q(&sim);
		relmr::JVec qdot = robot.get_qdot(&sim);
		q_list.push_back(q);
		static int print_count = 0;
		if(print_count++>100){
			cout << q_list.size()<<endl;
			print_count=0;
		}
 		sim.stepSimulation();
 		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
 		t = t+FIXED_TIMESTEP;	
		t_list.push_back(t);
 	}		
// Assuming relmr::JVec is an Eigen::VectorXd
std::vector<std::vector<double>> q_elements(6, std::vector<double>(N));  // Modify 6 to the dimension of your q vector if different

for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 6; ++j) {  // Modify loop limit to match your q dimension
        q_elements[j][i] = q_list[i][j];
    }
}

for (int i = 0; i < 6; ++i) {  // Modify loop limit to match your q dimension
    plot(t_list, q_elements[i])->line_width(2);
	hold(on);
}
show();
// Display the plot

    
}

