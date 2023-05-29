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
#include "../../include/MR_sim/MR_Indy7.h"
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
	sim.configureDebugVisualizer(COV_ENABLE_GUI,0);

	sim.resetSimulation();
	sim.setGravity( btVector3(0 , 0 ,-9.8));
	
	int robotId = sim.loadURDF("model/indy7.urdf");  
	int planeId = sim.loadURDF("model/plane.urdf");  
	Indy7 indy7(&sim,robotId);
	
	SE3 M0 = control.M;
	ScrewList Slist = control.Slist;
	ScrewList Blist = control.Blist;
	vector<mr::Matrix6d> Glist= control.Glist;	
	vector<mr::SE3> Mlist= control.Mlist;	

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
	q0<<0,0,-M_PI/2.0,0,-M_PI/2.0,0;
	qT<<0.5,0,-M_PI/2.0,0,-M_PI/2.0,0;
	indy7.resetQ(&sim,q0);
	int traj_flag = 0;
	double Tf = 10.0;
	int N = 1000;

    SE3 X0 = FKinSpace(M0,Slist,q0);
	SO3 R0 = TransToR(X0);
	Vector3d p = TransToP(X0);
	p(2) = p(2)+0.1;
	SE3 XT= RpToTrans(R0,p);
	Vector6d V0 = Vector6d::Zero();
	Vector6d VT = Vector6d::Zero();
	Vector6d dV0 = Vector6d::Zero();
	Vector6d dVT = Vector6d::Zero();
	
	vector<SE3> Xd_list ;
	vector<Vector6d> Vd_list ;
	vector<Vector6d> dVd_list ;
	mr::SE3 T=mr::SE3::Identity();
	mr::Jacobian Jb=mr::Jacobian::Zero();
	mr::Jacobian dJb=mr::Jacobian::Zero();
	LieScrewTrajectory(X0,XT,V0,VT,dV0,dVT,Tf,N,Xd_list,Vd_list,dVd_list);
	Matrix6d Kp = Matrix6d::Identity();
	Matrix6d Kd = Matrix6d::Identity();
	JVec q= indy7.getQ( &sim);
	JVec dq= indy7.getQdot( &sim);		
	JVec ddq= JVec::Zero();	
	for(int i = 0;i<N ;i++){

		JVec e = q_des-q;
		eint = eint+ e*dt;
		SE3 Xd = Xd_list.at(i);
		Vector6d Vd = Vd_list.at(i);
		Vector6d dVd = dVd_list.at(i);
		//FKinBody(M0, Blist, q ,dq, T, Jb,dJb);
		Jacobian Jb = JacobianBody(Blist,q);
		Vector6d V = Jb*dq;	

		Vector6d taskErr = se3ToVec(MatrixLog6(TransInv(T)*Xd));
		Vector6d taskErrDot = Adjoint(TransInv(T)*Xd)*Vd;


		JVec forcePD = Kp*taskErr+Kd*taskErrDot;
		JVec G = control.Gravity(q);
		MassMat M = mr::MassMatrix(q,Mlist,Glist,Slist);
		JVec C = mr::VelQuadraticForces(q, dq,Mlist,Glist, Slist);
		T =FKinBody(M0,Blist,q);
		dq = Jb.inverse()*Vd;
		JVec  edot = dq_des-dq;
		JVec ddq_ref =Kd*edot;
		//JVec torq = M*ddq_ref+C+G;
		//JVec torq = Jb.transpose()*forcePD+M*ddq+C+G;
		//dq = Jb.inverse()*Vd;
		EulerStep(q, dq, ddq, dt);                                  
		indy7.resetQ(&sim,q);
		//JVec forcePd = Kp*
		//JVec clacTorq = control.ComputedTorqueControl( q, dq, q_des, dq_des); // calcTorque
		//JVec clacPIDTorq = control.ComputedTorquePIDControl( q, dq, q_des, dq_des,eint); // calcTorque
		//JVec clacHinfTorq= control.HinfControl(  q, dq, q_des, dq_des,ddq_des,eint);

		static int print_count = 0;
		if(++print_count>10){
			cout<<se3ToVec(MatrixLog6(TransInv(T)*Xd)).transpose()<<endl;
			print_count = 0;
		}
		prev_dq = dq;
		//indy7.setTorques(&sim,  torq , MAX_TORQUES);
		sim.stepSimulation();
		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
		t = t+FIXED_TIMESTEP;	
	}
	while(1){
		JVec q= indy7.getQ( &sim);
		JVec gravTorq = control.Gravity( q);
		indy7.setTorques(&sim,  gravTorq , MAX_TORQUES);
		sim.stepSimulation();
		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
		t = t+FIXED_TIMESTEP;	
	}

	return -1;
    
}
