#include "DualArm.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "Bullet3Common/b3HashMap.h"
#include "Utils/b3Clock.h"

DualArm::DualArm(class b3RobotSimulatorClientAPI_NoDirect* sim,int rightId,int leftId){
    this->rightId = rightId;
    this->leftId = leftId;
    this->R = new Indy7(sim,this->rightId);
    this->L = new Indy7(sim,this->leftId);
}
relmr::JVec DualArm::get_q(class b3RobotSimulatorClientAPI_NoDirect* sim){
    relmr::JVec q = relmr::JVec::Zero();
    mr::JVec q_r = this->R->get_q(sim);
    mr::JVec q_l = this->L->get_q(sim);
    q.segment<6>(0)  = q_r;
    q.segment<6>(6)  = q_l;
    return q;
}
relmr::JVec DualArm::get_qdot(class b3RobotSimulatorClientAPI_NoDirect* sim){
    relmr::JVec qdot = relmr::JVec::Zero();
    mr::JVec qdot_r = this->R->get_qdot(sim);
    mr::JVec qdot_l = this->L->get_qdot(sim);
    qdot.segment<6>(0)  = qdot_r;
    qdot.segment<6>(6)  = qdot_l;
    return qdot;
}
relmr::JVec DualArm::get_q_rel(class b3RobotSimulatorClientAPI_NoDirect* sim){
    relmr::JVec q = relmr::JVec::Zero();
    mr::JVec q_r = this->R->get_q(sim);
    mr::JVec q_l = this->L->get_q(sim);
    q(0) = q_r(0);
    q(1) = q_r(1);
    q(2) = q_r(2);
    q(3) = q_r(3);
    q(4) = q_r(4);
    q(5) = q_r(5);
    q(6)  = -q_l(5);
    q(7)  = -q_l(4);
    q(8)  = -q_l(3);
    q(9)  = -q_l(2);
    q(10) = -q_l(1);
    q(11) = -q_l(0);
    return q;
}
relmr::JVec DualArm::get_qdot_rel(class b3RobotSimulatorClientAPI_NoDirect* sim){
    relmr::JVec qdot = relmr::JVec::Zero();
    mr::JVec qdot_r = this->R->get_qdot(sim);
    mr::JVec qdot_l = this->L->get_qdot(sim);
    qdot(0) = qdot_r(0);
    qdot(1) = qdot_r(1);
    qdot(2) = qdot_r(2);
    qdot(3) = qdot_r(3);
    qdot(4) = qdot_r(4);
    qdot(5) = qdot_r(5);
    qdot(6)  = -qdot_l(5);
    qdot(7)  = -qdot_l(4);
    qdot(8)  = -qdot_l(3);
    qdot(9)  = -qdot_l(2);
    qdot(10) = -qdot_l(1);
    qdot(11) = -qdot_l(0);
    return qdot;
}
DualArm::~DualArm(){
	
}
