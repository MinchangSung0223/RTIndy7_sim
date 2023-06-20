#ifndef DAULARM_SETUP_H
#define DAULARM_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "modern_robotics.h"
#include "modern_robotics_relative.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include "Indy7.h"

using namespace Eigen;
using namespace std;
class DualArm
{
	Indy7* R;
	Indy7* L;
	int leftId;
	int rightId;
public:
	DualArm(class b3RobotSimulatorClientAPI_NoDirect* sim,int rightId,int leftId);
	relmr::JVec get_q(class b3RobotSimulatorClientAPI_NoDirect* sim);
	relmr::JVec get_qdot(class b3RobotSimulatorClientAPI_NoDirect* sim);
	relmr::JVec get_q_rel(class b3RobotSimulatorClientAPI_NoDirect* sim);
	relmr::JVec get_qdot_rel(class b3RobotSimulatorClientAPI_NoDirect* sim);
	virtual ~DualArm();

};
#endif  //DAULARM_SETUP_H
