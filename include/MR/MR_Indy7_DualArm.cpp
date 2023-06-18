#include "MR_Indy7.h"
#include "MR_Indy7_DualArm.h"
bool ReadFromFile_(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData_(const char* filename,Json::Value &rootr){
	cout<<"START ReadMRData"<<endl;
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile_(filename, readBuffer, BufferLength)) {
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
 void MR_Indy7_DualArm::setq(mr::JVec q_l, mr::JVec q_r){
		for(int i = 0;i<JOINTNUM;i++){
			this->q[i] = -q_l[JOINTNUM-i-1];
			this->q[i+JOINTNUM] = q_r[i];
		}
		this->left_arm->q = q_l;
		this->right_arm->q = q_r;
 };
 void MR_Indy7_DualArm::setdq(mr::JVec dq_l, mr::JVec dq_r){
		for(int i = 0;i<JOINTNUM;i++){
			this->dq[i] = -dq_l[JOINTNUM-i-1];
			this->dq[i+JOINTNUM] = dq_r[i];
		}
		this->left_arm->dq = dq_l;
		this->right_arm->dq = dq_r;
 };

 void MR_Indy7_DualArm::getq(mr::JVec& q_l, mr::JVec& q_r){
		for(int i = 0;i<JOINTNUM;i++){
			q_l[JOINTNUM-i-1] = -this->q[i];
			q_r[i] = this->q[i+JOINTNUM];
		}
 }; 
 void MR_Indy7_DualArm::getdq(mr::JVec& dq_l, mr::JVec& dq_r){
		for(int i = 0;i<JOINTNUM;i++){
			dq_l[JOINTNUM-i-1] = -this->dq[i];
			dq_r[i] = this->dq[i+JOINTNUM];
		}
 }; 
 MR_Indy7_DualArm::MR_Indy7_DualArm(){
    this->jointnum = 12;
    this->left_arm = new MR_Indy7();
    this->right_arm = new MR_Indy7();
    this->left_arm->MRSetup();
    this->right_arm->MRSetup();
    this->left_arm->q = JVec::Zero();
    this->right_arm->q = JVec::Zero();
    this->q = relJVec::Zero();
    this->left_arm->g << 0 ,8.487,-4.9;
    this->right_arm->g << 0 ,8.487,-4.9;

 };
void MR_Indy7_DualArm::MRSetup(){
	Json::Value rootr;
	bool ret = ReadMRData_("MR_info.json",rootr);
	for(int i =0;i<6 ; i++){
		for(int j =0;j<this->jointnum;j++){
			this->Slist(i,j) = rootr["relSlist"][i][j].asDouble();
			this->Blist(i,j) = rootr["relBlist"][i][j].asDouble();
		}
	}	
    cout<<"=================relSlist================="<<endl;
    cout<<this->Slist<<endl;
    cout<<"=================relBlist================="<<endl;
    cout<<this->Blist<<endl;
	
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["relM"][i][j].asDouble();
		}
	}	

	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->Tbl(i,j) = rootr["Tbl"][i][j].asDouble();
		}
	}    
    cout<<"=================relM================="<<endl;
    cout<<this->M<<endl;    
    cout<<"=================Tbl================="<<endl;
    cout<<this->Tbl<<endl;    
	cout<<"END MRSetup"<<endl;

}
