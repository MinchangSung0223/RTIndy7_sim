#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include "../../include/MR/modern_robotics.h"
#include "../../include/MR/MR_Indy7.h"
using namespace std;
using namespace mr;
MR_Indy7 *control;
int main(){
	control=new MR_Indy7();
	control->MRSetup();    

    //-------------------------------------
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
    // bool ret = IKinBody(control->Blist, control->M, T,q, eomg, ev) ;
    //  ret = IKinSpace(control->Slist, control->M, T,q, eomg, ev) ;
    cout<<q.transpose()<<endl;
    JVec dq = JVec::Random();
    JVec ddq = JVec::Random();
    Vector3d g;
    g<<0,0,-9.8;
    Vector6d Ftip =Vector6d::Zero();
    cout<<control->Mlist.at(0)<<endl;
    JVec tau = InverseDynamics(q, dq, ddq,g,Ftip,control->Mlist,control->Glist,control->Slist);
    cout<<tau.transpose()<<endl;
    JVec ddq_next = ForwardDynamics(q,dq,ddq,g,Ftip,control->Mlist,control->Glist,control->Slist);  
    cout<<ddq_next.transpose()<<endl;    
    Jacobian Jb,dJb;
    SE3 retT;
    FKinBody(control->M,control->Blist,q ,dq,retT,Jb,dJb);
    cout<<"retT"<<endl;
    cout<<retT<<endl;
    cout<<"Jb"<<endl;
    cout<<Jb<<endl;
    cout<<"dJb"<<endl;
    cout<<dJb<<endl;    
    SE3 X0 =FKinBody(control->M,control->Blist,q);
    SE3 XT =X0;
    XT(0,3) = XT(0,3)+0.1;
    XT(1,3) = XT(1,3)+0.1;
    XT(2,3) = XT(2,3)+0.1;
    Vector6d V0 = Vector6d::Random();
    Vector6d VT = Vector6d::Random();
    Vector6d dV0 = Vector6d::Random();
    Vector6d dVT = Vector6d::Random();
    double Tf = 5;
    int N =5000;
    vector<SE3> Xd_list;
    vector<Vector6d> Vd_list;
    vector<Vector6d> dVd_list;
    LieScrewTrajectory(X0,XT,V0,VT,dV0,dVT,Tf,N,Xd_list,Vd_list,dVd_list);
    cout<<Xd_list.at(500)<<endl;
    return -1;
}