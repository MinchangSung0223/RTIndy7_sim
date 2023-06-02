#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include "../../include/LieRobotics/lie_robotics.h"
//#include "../../include/LieRobotics/LR_Indy7.h"
using namespace std;
using namespace lr;
//LR_Indy7 control;
int main(){
	//control=LR_Indy7();
//	control.LRSetup();    
    cout<<"LieRobotics Test"<<endl;
    SE3 T = SE3::Random();
    T.block<1,4>(3,0)<<0,0,0,1;
    //cout<<"V"<<endl;
    //cout<<V.transpose()<<endl;
    //cout<<"adV"<<endl;
    //cout<<ad(V)<<endl;
    //cout<<T<<endl;
    //cout<<Ad(T)<<endl;
    //cout<<TransInv(T)<<endl;
    Vector3d w =Vector3d::Random();
    so3 w_ = lr::VecToso3(w);
    Vector6d V=Vector6d::Random();
    se3 V_ = VecTose3(V);
    cout<<V_<<endl;
    auto start = std::chrono::high_resolution_clock::now();
    for( int i = 0;i<100000;i++){
        T =MatrixExp6(V_);
    }
        
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration = end - start;

    cout<<T<<endl;
    std::cout << "Execution time: " << duration.count()/100000.0 << " microseconds" << std::endl;

    // T.block<3,3>(0,0)=VecToso3(w);
    // cout<<w.transpose()<<endl;
    // cout<<""<<endl;
    // cout<<T<<endl;
    // cout<<""<<endl;
    // cout<<se3ToVec(T)<<endl;
    return -1;
}