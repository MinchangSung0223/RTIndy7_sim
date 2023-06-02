#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <vector>

#include "../../include/LieRobotics/lie_robotics.h"
#include "../../include/LieRobotics/LR_Indy7.h"
using namespace std;
using namespace lr;
LR_Indy7 *control;
int main(){
	control=new LR_Indy7();
	control->LRSetup();    
    JVec q = JVec::Random();
    cout<<q.transpose()<<endl;
    cout<<""<<endl;
    cout<<FKinBody(control->M,control->Blist,q)<<endl;
    cout<<FKinSpace(control->M,control->Blist,q)<<endl;
    cout<<JacobianBody(control->Blist,q)<<endl;
    cout<<JacobianSpace(control->Slist,q)<<endl;
    double eomg = 0.01;
    double ev = 0.001;
    SE3 T=FKinBody(control->M,control->Blist,q);
    T(0,3) = T(0,3)+0.1;
    bool ret = IKinBody(control->Blist, control->M, T,q, eomg, ev) ;
     ret = IKinSpace(control->Slist, control->M, T,q, eomg, ev) ;
    cout<<q.transpose()<<endl;
    JVec dq = JVec::Random();
    JVec ddq = JVec::Random();
    Vector3d g;
    g<<0,0,-9.8;
    Vector6d Ftip =Vector6d::Zero();
    cout<<control->Mlist.at(0)<<endl;
    //JVec tau = InverseDynamics(q, dq, ddq,g,Ftip,control->Mlist,control->Glist,control->Slist);
    //cout<<tau<<endl;
    return -1;
}