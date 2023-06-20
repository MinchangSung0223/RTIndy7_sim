#ifndef INDY7_SETUP_H
#define INDY7_SETUP_H

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

public:
	DualArm(class b3RobotSimulatorClientAPI_NoDirect* sim);
	virtual ~DualArm();

};
#endif  //DAULARM_SETUP_H
