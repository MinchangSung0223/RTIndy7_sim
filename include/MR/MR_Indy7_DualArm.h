#ifndef MR_INDY7_DUALARM_H
#define MR_INDY7_DUALARM_H
#include "include/NRMKSDK/json/json/json.h"
#include "iostream"
#include "MR_Indy7.h"

#include "modern_robotics.h"

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace mr;
#define RELJOINTNUM 12
class MR_Indy7_DualArm {
public:
    MR_Indy7_DualArm();  // Constructor
    typedef Eigen::Matrix<double, RELJOINTNUM, 1> relJVec;
    typedef Eigen::Matrix<double, 6, RELJOINTNUM> relScrewList;
    MR_Indy7* left_arm;
    MR_Indy7* right_arm;
    unsigned int jointnum;
    relScrewList Slist;
    relScrewList Blist;

    mr::SE3 M;
    mr::SE3 Tbl;

    relJVec q;
    relJVec dq;
    relJVec ddq;


    relJVec q_des;
    relJVec dq_des;
    relJVec ddq_des;

    mr::Vector3d g;
    mr::JVec  torq;

    mr::Matrix6d Kp;
    mr::Matrix6d Kv;
    mr::Matrix6d Ki;

    mr::Matrix6d Hinf_Kp;
    mr::Matrix6d Hinf_Kv;
    mr::Matrix6d Hinf_Ki;
    mr::Matrix6d Hinf_K_gamma;

    void MRSetup();
    //JVec Gravity( JVec q);
    //JVec ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des);
    //void saturationMaxTorque(JVec &torque, JVec MAX_TORQUES);
    //JVec ComputedTorquePIDControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec& eint);
    //JVec HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec& eint);
};

#endif // MR_INDY7_DUALARM_H
