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
#include "MR_Indy7.h"
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
MR_Indy7 control;

int main()
{
	control=MR_Indy7();
	control.MRSetup();
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
	
	int robotId = sim.loadURDF("model/indy7.urdf");  
	Indy7 indy7(&sim,robotId);
	
	double t = 0;
	double dt= FIXED_TIMESTEP;
	JVec MAX_TORQUES;
//	MAX_TORQUES<<431.97,431.97,197.23,79.79,79.79,79.79;
	MAX_TORQUES<<1000,1000,1000,1000,1000,1000;

	JVec q_des=JVec::Zero();
	q_des(0) = 1.0;
	q_des(1) = 1.0;
	q_des(2) = 1.0;
	q_des(3) = 1.0;
	q_des(4) = 1.0;
	q_des(5) = 1.0;
	
	JVec dq_des=JVec::Zero();	
	JVec ddq_des=JVec::Zero();	
	JVec eint = JVec::Zero();
	JVec prev_dq = JVec::Zero();
	JVec q0 = JVec::Zero();
	JVec qT = JVec::Zero();
	JVec qT1 = JVec::Zero();
	JVec qT2 = JVec::Zero();
	qT<<1.0,1.0,1.0,1.0,1.0,1.0;
	qT1<<0,0,0,0,0,0;
	qT2<<1.0,1.0,1.0,1.0,1.0,1.0;
	int traj_flag = 0;
	double Tf = 1.0;
	vector<JVec> q_des_list;
	vector<JVec> dq_des_list;
	vector<JVec> ddq_des_list;
	JointTrajectoryList(q0, qT, Tf, int(Tf/dt), 5 , q_des_list, dq_des_list, ddq_des_list) ;
	JointTrajectoryList(qT, qT1, Tf, int(Tf/dt), 5 , q_des_list, dq_des_list, ddq_des_list) ;
	JointTrajectoryList(qT1, qT2, Tf, int(Tf/dt), 5 , q_des_list, dq_des_list, ddq_des_list) ;
	JointTrajectoryList(qT2, q0, Tf, int(Tf/dt), 5 , q_des_list, dq_des_list, ddq_des_list) ;
	int i =0;
	while(1){
		JVec q= indy7.get_q( &sim);
		JVec dq= indy7.get_qdot( &sim);		
		JVec ddq= (prev_dq-dq)/dt;
		JVec e = q_des-q;
		eint = eint+ e*dt;
		q_des = q_des_list.at(i);
		dq_des = dq_des_list.at(i);
		ddq_des = ddq_des_list.at(i++);
		if(traj_flag ==0){
				q0 = q;
				traj_flag =1;
			}
		

		JVec gravTorq = control.Gravity( q);
		JVec clacTorq = control.ComputedTorqueControl( q, dq, q_des, dq_des); // calcTorque
		JVec clacPIDTorq = control.ComputedTorquePIDControl( q, dq, q_des, dq_des,eint); // calcTorque
		JVec clacHinfTorq= control.HinfControlSim(  q, dq, q_des, dq_des,ddq_des,eint);

		static int print_count = 0;
		if(++print_count>100){
			cout<<t<<endl;
			cout<<"e   :"<<e.transpose()<<endl;
			cout<<"eint:"<<eint.transpose()<<endl;
			print_count = 0;
		}
		prev_dq = dq;
		indy7.set_torque(&sim,  clacHinfTorq , MAX_TORQUES);
		sim.stepSimulation();
		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
		t = t+FIXED_TIMESTEP;	
	}

	return -1;
    
}
