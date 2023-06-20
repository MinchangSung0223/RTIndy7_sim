#ifndef INDY7_SETUP_H
#define INDY7_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "modern_robotics.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
using namespace mr;

class Indy7
{
	int robotId;
	int actuated_joint_num;
	int eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
		
public:

	Indy7(class b3RobotSimulatorClientAPI_NoDirect* sim,int robotId);
	void set_torque(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec  torques ,JVec  max_torques );
	JVec get_q(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	JVec get_qdot(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	SE3 get_eef_pose(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	void reset_q(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec q);
	Vector6d get_FT(class b3RobotSimulatorClientAPI_NoDirect* sim);
	void apply_ext_FT(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec FT);
	int get_actuated_joint_num(){
		return this->actuated_joint_num;
	};
	
	virtual ~Indy7();

};
#endif  //INDY7_SETUP_H
