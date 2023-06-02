#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include "../../include/MR/modern_robotics.h"
#include "../../include/MR/MR_Indy7.h"
using namespace std;
using namespace mr;
MR_Indy7 control;
int main(){
	control=MR_Indy7();
	control.MRSetup();    
    JVec q = JVec::Random();
    cout<<q.transpose()<<endl;
    cout<<""<<endl;
    cout<<FKinBody(control.M,control.Blist,q)<<endl;
    cout<<FKinSpace(control.M,control.Blist,q)<<endl;
    cout<<JacobianBody(control.Blist,q)<<endl;
    cout<<JacobianSpace(control.Slist,q)<<endl;
    double eomg = 0.01;
    double ev = 0.001;
    SE3 T=FKinBody(control.M,control.Blist,q);
    T(0,3) = T(0,3)+0.1;
    bool ret = IKinBody(control.Blist, control.M, T,q, eomg, ev) ;
     ret = IKinSpace(control.Slist, control.M, T,q, eomg, ev) ;
    cout<<q.transpose()<<endl;
    return -1;
}