#ifndef LR_INDY7_H
#define LR_INDY7_H
#include "include/NRMKSDK/json/json/json.h"
#include "iostream"
#include "lie_robotics.h"
#include <vector>

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace lr;
class LR_Indy7 {
public:
    LR_Indy7();  // Constructor
    lr::ScrewList Slist;
    lr::ScrewList Blist;
    lr::SE3 M;

	vector<lr::Matrix6d> Glist;	
	vector<lr::SE3> Mlist;	
    lr::JVec q;
    lr::JVec dq;
    lr::JVec ddq;

    lr::JVec q_des;
    lr::JVec dq_des;
    lr::JVec ddq_des;
    

    lr::Vector3d g;
    lr::JVec  torq;

    lr::Matrix6d Kp;
    lr::Matrix6d Kv;
    lr::Matrix6d Ki;

    lr::Matrix6d Hinf_Kp;
    lr::Matrix6d Hinf_Kv;
    lr::Matrix6d Hinf_Ki;
    lr::Matrix6d Hinf_K_gamma;

    void LRSetup();
};

#endif // LR_INDY7_H
