#include "LR_Indy7.h"

bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData(const char* filename,Json::Value &rootr){
	cout<<"START ReadMRData"<<endl;
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile(filename, readBuffer, BufferLength)) {
		std::cout<<"Failed"<<std::endl;
		return -1;
	}
	std::string config_doc = readBuffer;

	Json::Reader reader;
	bool parsingSuccessful = reader.parse(config_doc,rootr);
	if ( !parsingSuccessful ) { 
		std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
		return -1;
		
	}
    cout<<"END ReadMRData"<<endl;

    return 1;
}


LR_Indy7::LR_Indy7() {
    // Constructor implementation

	this->Slist.resize(6,6);
	this->Blist.resize(6,6);	
	this->Glist;	
	this->Mlist;			
	this->M.resize(4,4);	    
    this->q.resize(6);	
    this->q_des.resize(6);	
    this->dq_des.resize(6);	
    this->ddq_des.resize(6);	
    this->dq.resize(6);	
    this->g.resize(3);
    this->torq.resize(6);

    this->g<<0,0,-9.8;
    this->Kp = lr::Matrix6d::Zero();
    this->Kv = lr::Matrix6d::Zero();
    this->Ki = lr::Matrix6d::Zero();
    this->Hinf_Kp = lr::Matrix6d::Zero();
    this->Hinf_Kv = lr::Matrix6d::Zero();
    this->Hinf_K_gamma = lr::Matrix6d::Zero();
    Vector6d invL2sqr=Vector6d::Zero();
    invL2sqr<<800.0,600.0,500.0,500.0,500.0,600.0;
    Vector6d K=Vector6d::Zero();
    K<<50,30,30,3,3,0.1;
    for (int i=0; i<6; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 4:
              Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;

            break;
        }
    }
   for (int i=0; i<6; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 2:
            Kp(i,i) = 40.0;
            Kv(i,i) = 30.0;
            Ki(i,i)=5.0;
            break;
        case 3:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 4:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 5:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=1.0;
            break;
        }
    }
}

// JVec MR_Indy7::Gravity( JVec q){
//          return lr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist) ; 
// }
// void MR_Indy7::saturationMaxTorque(JVec &torque, JVec MAX_TORQUES){
//     for(int i =0;i<JOINTNUM;i++){
//         if(abs(torque(i))> MAX_TORQUES(i)){
//             if(torque(i)>0) torque(i) = MAX_TORQUES(i);
//             else torque(i) = -MAX_TORQUES(i);
//         }
//     }
// }
// JVec MR_Indy7::ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des){
//     JVec e = q_des-q;
//     JVec edot = dq_des-dq;
//     MassMat Mmat = lr::MassMatrix(q,this->Mlist, this->Glist, this->Slist);
//     JVec H=InverseDynamics(q, dq, JVec::Zero(),this->g,Vector6d::Zero(), this->Mlist,this->Glist, this->Slist);
//     JVec ddq_ref = Kv*edot+Kp*e;
//     JVec torq = Mmat*ddq_ref+H;
//     return torq;
// }
// JVec MR_Indy7::ComputedTorquePIDControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec& eint){
//     JVec e = q_des-q;
//     JVec edot = dq_des-dq;
//     MassMat Mmat = lr::MassMatrix(q,this->Mlist, this->Glist, this->Slist);
//     JVec H=InverseDynamics(q, dq, JVec::Zero(),this->g,Vector6d::Zero(), this->Mlist,this->Glist, this->Slist);
//     JVec ddq_ref = Kv*edot+Kp*e+Ki*eint;
//     JVec torq = Mmat*ddq_ref+H;
//     return torq;
// }
// JVec MR_Indy7::HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec& eint){
//     JVec e = q_des-q;
//     JVec edot = dq_des-dq;
//     MassMat Mmat = lr::MassMatrix(q,this->Mlist, this->Glist, this->Slist);
//     JVec C = lr::VelQuadraticForces(q, dq,this->Mlist, this->Glist, this->Slist);
//     JVec G = lr::GravityForces(q,this->g,this->Mlist, this->Glist, this->Slist) ; 
//     JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
//     JVec torq = Mmat*ddq_ref+C+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
//     return torq;
// }

void LR_Indy7::LRSetup(){
	Json::Value rootr;
	bool ret = ReadMRData("LR_info.json",rootr);
    if(ret!=0) cout<<"NO LR_info.json"<<endl;
    ScrewList Blist,Slist;
	for(int i =0;i<6 ; i++){
		for(int j =0;j<6;j++){
			this->Slist(i,j) = rootr["Slist"][i][j].asDouble();
			this->Blist(i,j) = rootr["Blist"][i][j].asDouble();
		}
	}	
    //this->Blist.block<3,JOINTNUM>(0,0) =Blist_.block<3,JOINTNUM>(3,0);
    //this->Blist.block<3,JOINTNUM>(3,0) = Blist_.block<3,JOINTNUM>(0,0);
    
//    this->Slist.block<3,JOINTNUM>(0,0) = Slist_.block<3,JOINTNUM>(3,0);
    //this->Slist.block<3,JOINTNUM>(3,0) = Slist_.block<3,JOINTNUM>(0,0);

    cout<<"=================Slist================="<<endl;
    cout<<this->Slist<<endl;
    cout<<"=================Blist================="<<endl;
    cout<<this->Blist<<endl;
	for(int i = 0;i< rootr["Mlist"].size(); i++){
		MatrixXd M = MatrixXd::Identity(4,4);
		for(int j = 0;j< rootr["Mlist"][0].size(); j++){
			for(int k = 0;k< rootr["Mlist"][0][0].size(); k++){
				M(j,k) = rootr["Mlist"][i][j][k].asDouble();
			}
		}
        cout<<"=================M"<<i<<"============================"<<endl;
        cout<<M<<endl;

		char str[50];		
		this->Mlist.push_back(M);
	}
	for(int i = 0;i< rootr["Glist"].size(); i++){
		MatrixXd G = MatrixXd::Identity(6,6);
        //MatrixXd G_=MatrixXd::Identity(6,6);
		for(int j = 0;j< rootr["Glist"][0].size(); j++){
			for(int k = 0;k< rootr["Glist"][0][0].size(); k++){
				G(j,k) = rootr["Glist"][i][j][k].asDouble();
			}
		}
        cout<<"=================G"<<i<<"============================"<<endl;
        cout<<G<<endl;

       // G_.block<3,3>(0,0) = G.block<3,3>(3,3);
        //G_.block<3,3>(3,3) = G.block<3,3>(0,0);
		char str[50];		
		this->Glist.push_back(G);	}	
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["M"][i][j].asDouble();
		}
	}	
    cout<<"=================M================="<<endl;
    cout<<this->M<<endl;    
	cout<<"END MRSetup"<<endl;

}
