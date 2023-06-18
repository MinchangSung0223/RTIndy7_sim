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
#include "../../include/MR/modern_robotics_relative.h"
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
	double t = 0;
	JVec MAX_TORQUES;
	MAX_TORQUES<<1000,1000,1000,1000,1000,1000;
	JVec HOME_LEFT,HOME_RIGHT;
	HOME_LEFT<< -0.1455808405866516 , -0.8133711809728681  ,-1.3075236414349891 , -0.6146456270149877  ,-1.0306524888727113  ,0.06930112107483291 ;
	HOME_RIGHT<< 0.1455808405866516 , 0.8133711809728681  ,1.3075236414349891 , 0.6146456270149877  ,1.0306524888727113 ,0 ;
	//HOME_LEFT<< -0.166785,-0.112846,-2.803911,0.061847,-0.224256,0.5;
	//HOME_RIGHT<< 0.147439,0.258307,-2.698818,0.281465,-0.200132,-0.849611;
	left_arm.resetQ(&sim,HOME_LEFT);
	right_arm.resetQ(&sim,HOME_RIGHT);
	dualarm.left_arm->q = left_arm.getQ( &sim);
	dualarm.right_arm->q = right_arm.getQ( &sim);
	relmr::JVec q = relmr::JVec::Zero();
	relmr::JVec dq = relmr::JVec::Zero();
	relmr::JVec ddq = relmr::JVec::Zero();
	SE3 Xd ;
	Xd<<-1,0 ,0 ,0,
	     0, 1, 0 ,0 ,
		 0,0,-1,0.4,
		 0,0,0,1;
	Vector6d Vd = Vector6d::Zero();
	int print_count = 0;
	double dt =FIXED_TIMESTEP;
	dualarm.right_arm->e =mr::JVec::Zero();
	dualarm.left_arm->e =mr::JVec::Zero();
	dualarm.right_arm->eint =mr::JVec::Zero();
	dualarm.left_arm->eint =mr::JVec::Zero();
	
	dualarm.ddq_des = relmr::JVec::Zero();
	dualarm.right_arm->dq_des=mr::JVec::Zero();
	dualarm.left_arm->dq_des=mr::JVec::Zero();
	dualarm.right_arm->ddq_des=mr::JVec::Zero();
	dualarm.left_arm->ddq_des=mr::JVec::Zero();
	dualarm.right_arm->q_des=HOME_RIGHT;
	dualarm.left_arm->q_des=HOME_LEFT;

	cout<<dualarm.right_arm->q_des <<endl;
	cout<<dualarm.right_arm->q <<endl;
	cout<<dualarm.right_arm->e <<endl;
//	b3Clock::usleep(10000*1000. * 1000. * FIXED_TIMESTEP);
	mr::JVec eint=mr::JVec::Zero();
	while(1){
		dualarm.left_arm->q = left_arm.getQ( &sim);
		dualarm.right_arm->q = right_arm.getQ( &sim);
		dualarm.left_arm->dq = left_arm.getQdot( &sim);
		dualarm.right_arm->dq = right_arm.getQdot( &sim);

		dualarm.setq(dualarm.left_arm->q,dualarm.right_arm->q);
		dualarm.setdq(dualarm.left_arm->dq,dualarm.right_arm->dq);
		


		SE3 T;
		relmr::Jacobian Jb;
		relmr::Jacobian dJb;
		relmr::FKinBody(dualarm.M,dualarm.Blist, dualarm.q ,dualarm.dq, T, Jb,dJb);
		Vector6d V = Jb*dualarm.dq;	
		Vector6d Xe = se3ToVec(MatrixLog6(TransInv(T)*Xd));
		Vector6d Ve = Adjoint(TransInv(T)*Xd)*Vd-V;
		Vector6d forcePD = (1000.0*Xe+100*Ve)/dt;
		relmr::pinvJacobian pinvJb= Jb.transpose()*(Jb*Jb.transpose()).inverse();
		dualarm.dq_des =pinvJb*forcePD;
		relmr::EulerStep(dualarm.q_des ,dualarm.dq_des  ,dualarm.ddq_des , dt);

		
		dualarm.getq(dualarm.left_arm->q_des,dualarm.right_arm->q_des);
		dualarm.getdq(dualarm.left_arm->dq_des,dualarm.right_arm->dq_des);
		//left_arm.resetQ(&sim, dualarm.left_arm->q );
		//right_arm.resetQ(&sim, dualarm.right_arm->q );		

		//JVec gravLeft = dualarm.left_arm->Gravity( dualarm.left_arm->q);
		//JVec gravRight = dualarm.right_arm->Gravity( dualarm.right_arm->q);

		dualarm.left_arm->eint += dualarm.left_arm->e*dt;
		dualarm.right_arm->eint += dualarm.right_arm->e*dt;

		JVec tau_l= dualarm.left_arm->HinfControlSim(  dualarm.left_arm->q, dualarm.left_arm->dq,
		dualarm.left_arm->q_des, dualarm.left_arm->dq_des,dualarm.left_arm->ddq_des,dualarm.left_arm->eint);
		JVec tau_r= dualarm.right_arm->HinfControlSim(  dualarm.right_arm->q, dualarm.right_arm->dq,
		dualarm.right_arm->q_des, dualarm.right_arm->dq_des,dualarm.right_arm->ddq_des,dualarm.right_arm->eint);
	
 		left_arm.setTorques(&sim,   tau_l , MAX_TORQUES);
		right_arm.setTorques(&sim,  tau_r , MAX_TORQUES);


		if(++print_count>10){
			cout<<"r_dq_des:"<<dualarm.right_arm->dq_des.transpose()<<endl;
			//cout<<dualarm.left_arm->e.transpose()<<endl;
			print_count = 0;
		}
 		sim.stepSimulation();
 		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
 		t = t+FIXED_TIMESTEP;	
 	}		

    
}
