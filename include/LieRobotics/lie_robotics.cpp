
#include "lie_robotics.h"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
#include <chrono>

# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

namespace lr {
	Matrix6d ad(const Vector6d& V) {
		Matrix6d result=Matrix6d::Zero();
		
		result<<      0, -V(5),  V(4),  0,-V(2),V(1),
			       V(5),     0, -V(3),  V(2),0,-V(0),
				  -V(4),  V(3),     0,  -V(1),V(0),0,
				   0,    0,   0, 0 ,-V(5) ,V(4),
				   0,    0,   0, V(5) ,0 ,-V(3),
				   0,	 0,	  0,-V(4),V(3) ,0;
		return result;
	}    
	Matrix6d Ad(const SE3& T) {
		Matrix6d ad_ret = Matrix6d::Zero();
		ad_ret<<T(0,0),T(0,1),T(0,2),T(1,3)*T(2,0) - T(2,3)*T(1,0), T(1,3)*T(2,1) - T(2,3)*T(1,1), T(1,3)*T(2,2) - T(2,3)*T(1,2),
				T(1,0),T(1,1),T(1,2),T(2,3)*T(0,0) - T(0,3)*T(2,0), T(2,3)*T(0,1) - T(0,3)*T(2,1), T(2,3)*T(0,2) - T(0,3)*T(2,2),
				T(2,0),T(2,1),T(2,2),T(0,3)*T(1,0) - T(1,3)*T(0,0), T(0,3)*T(1,1) - T(1,3)*T(0,1), T(0,3)*T(1,2) - T(1,3)*T(0,2),
				0,0,0,T(0,0),T(0,1),T(0,2),
				0,0,0,T(1,0),T(1,1),T(1,2),
				0,0,0,T(2,0),T(2,1),T(2,2);
		return ad_ret;
	}
	// Matrix6d AdInv(const SE3& T) {
	// 	Matrix6d ad_ret = Ad(TransInv(T));
	// 	return ad_ret;
	// }				
	Matrix6d AdInv(const SE3& T) {
		Matrix6d ad_ret = Matrix6d::Zero();
		ad_ret<<T(0,0), T(1,0), T(2,0), T(1,3)*T(2,0) - T(1,0)*T(2,3), T(0,0)*T(2,3) - T(0,3)*T(2,0), T(0,3)*T(1,0) - T(0,0)*T(1,3),
				T(0,1), T(1,1), T(2,1), T(1,3)*T(2,1) - T(1,1)*T(2,3), T(0,1)*T(2,3) - T(0,3)*T(2,1), T(0,3)*T(1,1) - T(0,1)*T(1,3),
				T(0,2), T(1,2), T(2,2), T(1,3)*T(2,2) - T(1,2)*T(2,3), T(0,2)*T(2,3) - T(0,3)*T(2,2), T(0,3)*T(1,2) - T(0,2)*T(1,3),
				0, 0, 0, T(0,0), T(1,0), T(2,0),
				0, 0, 0, T(0,1), T(1,1), T(2,1),
				0, 0, 0, T(0,2), T(1,2), T(2,2);
		return ad_ret;
	}	
	SE3 TransInv(const SE3& T) {
		SE3 ret=SE3::Identity();
		ret<<T(0,0), T(1,0), T(2,0), - T(0,0)*T(0,3) - T(1,0)*T(1,3) - T(2,0)*T(2,3),
		 	 T(0,1), T(1,1), T(2,1), - T(0,1)*T(0,3) - T(1,1)*T(1,3) - T(2,1)*T(2,3), 
			 T(0,2), T(1,2), T(2,2), - T(0,2)*T(0,3) - T(1,2)*T(1,3) - T(2,2)*T(2,3),
			 0,0,0,1;
		return ret;
	}	
	
	SO3 TransToR(const SE3& T) {
		return T.block<3,3>(0,0);
	}	
	Vector3d TransToP(const SE3& T) {
		return Vector3d(T(0,3), T(1,3),T(2,3));
	}		

	se3 VecTose3(const Vector6d& V) {
		SE3 m_ret;
		m_ret << 0,-V(5),V(4),V(0),
		         V(5),0,-V(3),V(1),
				-V(4),V(3),0,V(2),
				 0 ,0 ,0,1;
		return m_ret;
	}
	so3 VecToso3(const Vector3d& omg) {
		so3 m_ret;
		m_ret << 0, -omg(2), omg(1),
			omg(2), 0, -omg(0),
			-omg(1), omg(0), 0;
		return m_ret;
	}

	Vector3d so3ToVec(const so3& so3mat) {
		Vector3d v_ret(so3mat(2, 1), so3mat(0, 2), so3mat(1, 0));
		return v_ret;
	}
	Vector6d se3ToVec(const se3& T) {
		Vector6d m_ret;
		m_ret<<T(0,3),T(1,3),T(2,3),T(2,1),-T(2,0),T(1,0);
		return m_ret;
	}	
	bool NearZero(const double val) {
		return (abs(val) < .000001);
	}	

	SO3 MatrixExp3(const so3& so3mat) {
		Vector3d omgtheta = so3ToVec(so3mat);
		double theta = omgtheta.norm();
		if (NearZero(so3mat.norm())) {
			return SO3::Identity();
		}
		else {			
			Matrix3d omgmat = so3mat / theta;
			Matrix3d omgmatomgmat;
			double w3 = omgmat(1,0);
			double w2 = omgmat(0,2);
			double w1 = omgmat(2,1);
			omgmatomgmat(0,0) = -w2*w2-w3*w3;
			omgmatomgmat(1,1) = -w1*w1-w3*w3;
			omgmatomgmat(2,2) = -w1*w1-w2*w2;		
			omgmatomgmat(0,1) = omgmatomgmat(1,0) = w1*w2;
			omgmatomgmat(0,2) = omgmatomgmat(2,0) =w1*w3;			
			omgmatomgmat(1,2) = omgmatomgmat(2,1) =w2*w3;			
			return SO3::Identity() + sin(theta) * omgmat + ((1 - cos(theta)) * (omgmatomgmat));
		}
	}	


	SE3 MatrixExp6(const se3& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		so3 se3mat_cut = se3mat.block<3, 3>(0, 0);
		Vector3d omgtheta = so3ToVec(se3mat_cut);
		SE3 m_ret= SE3::Identity();
		double theta = omgtheta.norm();
		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			m_ret.block<3,3>(0,0) = Matrix3d::Identity();
			m_ret.block<3,1>(0,3) = Vector3d(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			Matrix3d omgmat = se3mat_cut / theta;
			Matrix3d omgmatomgmat;
			double w3 = omgmat(1,0);
			double w2 = omgmat(0,2);
			double w1 = omgmat(2,1);
			omgmatomgmat(0,0) = -w2*w2-w3*w3;
			omgmatomgmat(1,1) = -w1*w1-w3*w3;
			omgmatomgmat(2,2) = -w1*w1-w2*w2;			
			omgmatomgmat(0,1) = omgmatomgmat(1,0) = w1*w2;
			omgmatomgmat(0,2) = omgmatomgmat(2,0) =w1*w3;
			omgmatomgmat(1,2) = omgmatomgmat(2,1) =w2*w3;
			Matrix3d expExpand = Matrix3d::Identity()* theta + (1 - cos(theta)) * omgmat + ((theta - sin(theta)) * (omgmatomgmat));
			Vector3d linear(se3mat(0, 3)/theta, se3mat(1, 3)/theta, se3mat(2, 3)/theta);
			Vector3d GThetaV(expExpand(0,0)*linear(0)+expExpand(0,1)*linear(1)+expExpand(0,2)*linear(2),
			expExpand(1,0)*linear(0)+expExpand(1,1)*linear(1)+expExpand(1,2)*linear(2),
			expExpand(2,0)*linear(0)+expExpand(2,1)*linear(1)+expExpand(2,2)*linear(2));
			m_ret.block<3,3>(0,0) = Matrix3d::Identity() + sin(theta) * omgmat + ((1 - cos(theta)) * (omgmatomgmat));
			m_ret.block<3,1>(0,3) = GThetaV;
			return m_ret;
		}
	}

	so3 MatrixLog3(const SO3& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		so3 m_ret =so3::Zero();	
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / sqrt(2 * (1 + R(2, 2))))*Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / sqrt(2 * (1 + R(1, 1))))*Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / sqrt(2 * (1 + R(0, 0))))*Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}	

	se3 MatrixLog6(const SE3& T) {
		se3 m_ret(4, 4);
		SO3 R = TransToR(T);
		Vector3d p = TransToP(T);
		so3 omgmat = MatrixLog3(R);

		if (NearZero(omgmat.norm())) {
			m_ret << SO3::Zero(), p,
				0, 0, 0, 0;
		}
		else {
			double theta = acos((R.trace() - 1) / 2.0);
			Matrix3d logExpand1 = SO3::Identity() - omgmat / 2.0;
			Matrix3d logExpand2 = (1.0 / theta - 1.0 / tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*p,
				0, 0, 0, 0;
		}
		return m_ret;
	}	

	SE3 FKinSpace(const SE3& M, const ScrewList& Slist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = (JOINTNUM - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}	
	SE3 FKinBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = 0; i < JOINTNUM; i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}	
	Jacobian JacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = Slist;	
		SE3 T = SE3::Identity();
		for (int i = 1; i < JOINTNUM; i++) {
			T *= MatrixExp6(VecTose3(Slist.col(i - 1) * thetaList(i - 1)));
			Js.col(i) = Ad(T) * Slist.col(i);
		}
		return Js;
	}	
	Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = Blist;
		SE3 T = SE3::Identity();
		for (int i = JOINTNUM -2; i >= 0; i--) {
			T *= MatrixExp6(VecTose3(-1 * Blist.col(i + 1) * thetaList(i + 1)));
			Jb.col(i) = Ad(T) * Blist.col(i);
		}
		return Jb;
	}		
	SE3 RpToTrans(const Matrix3d& R, const Vector3d& p) {
		SE3 m_ret=SE3::Identity();
		m_ret.block<3,3>(0,0) = R;
		m_ret.block<3,1>(0,3) = p;
		return m_ret;
	}			
	bool IKinBody(const ScrewList& Blist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 Tfk = FKinBody(M, Blist, thetalist);
		SE3 Tdiff = TransInv(Tfk)*T;
		Vector6d Vb = se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vb(3), Vb(4), Vb(5));
		Vector3d linear(Vb(0), Vb(1), Vb(2));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Jb_;
		Eigen::MatrixXd Jb;
		while (err && i < maxiterations) {
			Jb_ = JacobianBody(Blist, thetalist);
			Jb = Eigen::Map<Eigen::MatrixXd>(Jb_.data(),6,JOINTNUM);
						
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			Tfk = FKinBody(M, Blist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vb = se3ToVec(MatrixLog6(Tdiff));
			angular = Vector3d(Vb(3), Vb(4), Vb(5));
			linear = Vector3d(Vb(0), Vb(1), Vb(2));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}
	bool IKinSpace(const ScrewList& Slist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 Tfk = FKinSpace(M, Slist, thetalist);
		SE3 Tdiff = TransInv(Tfk)*T;
		Vector6d Vs = Ad(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vs(3), Vs(4), Vs(5));
		Vector3d linear(Vs(0), Vs(1), Vs(2));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Js_;
		Eigen::MatrixXd Js;
		while (err && i < maxiterations) {
			Js_ = JacobianSpace(Slist, thetalist);
			Js = Eigen::Map<Eigen::MatrixXd>(Js_.data(),6,JOINTNUM);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Ad(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Vector3d(Vs(3), Vs(4), Vs(5));
			linear = Vector3d(Vs(0), Vs(1), Vs(2));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}	

	// JVec InverseDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
	// 								const Vector3d& g, const Vector6d& Ftip, const vector<SE3>& Mlist,
	// 								const vector<Matrix6d>& Glist, const ScrewList& Slist) {
	//     // the size of the lists
	// 	int n = JOINTNUM;

	// 	SE3 Mi = SE3::Identity();
	// 	Matrix6xn Ai = Matrix6xn::Zero();
	// 	vector<ScrewList> AdTi;
	// 	for (int i = 0; i < n+1; i++) {
	// 		AdTi.push_back(Matrix6d::Zero());
	// 	}
	// 	Matrix6xn_1 Vi = Matrix6xn_1::Zero();    // velocity
	// 	Matrix6xn_1 Vdi = Matrix6xn_1::Zero();   // acceleration

	// 	Vdi.block(3, 0, 3, 1) = - g;
	// 	AdTi[n] = Ad(TransInv(Mlist[n]));
	// 	Vector6d Fi = Ftip;

	// 	JVec taulist = JVec::Zero();

	// 	// forward pass
	// 	for (int i = 0; i < n; i++) {
	// 		Mi = Mi * Mlist[i];
	// 		Ai.col(i) = Ad(TransInv(Mi))*Slist.col(i);

	// 		AdTi[i] = Ad(MatrixExp6(VecTose3(Ai.col(i)*-thetalist(i)))
	// 		          * TransInv(Mlist[i]));

	// 		Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
	// 		Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
	// 					   + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
	// 	}

	// 	// backward pass
	// 	for (int i = n-1; i >= 0; i--) {
	// 		Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
	// 		     - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
	// 		taulist(i) = Fi.transpose() * Ai.col(i);
	// 	}
	// 	return taulist;
	// }

}

