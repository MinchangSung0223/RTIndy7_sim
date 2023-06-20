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
	void setTorques(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec  torques ,JVec  max_torques );
	JVec getQ(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	JVec getQdot(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	SE3 getEEFPose(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	void resetQ(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec q);
	Vector6d getFTsensor(class b3RobotSimulatorClientAPI_NoDirect* sim);
	void applyExtFT(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec FT);
	int getActuatedJointNum(){
		return this->actuated_joint_num;
	};
	
	virtual ~Indy7();

};
#endif  //INDY7_SETUP_H
