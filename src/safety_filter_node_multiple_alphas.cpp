#include "segway_sim/safety_filter_node.hpp"

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

ros::Subscriber sub_state_;
ros::Subscriber sub_inputDes_;

ros::Publisher pub_inputAct_;
ros::Publisher pub_info_;
ros::Publisher pub_backupTraj_;

segway_sim::input inputDes_;
segway_sim::input inputAct_;
segway_sim::state stateCurrent_;
segway_sim::filterInfo filter_info_;
nav_msgs::Path backTrajMsg_;

int32_t passThrough_;
uint32_t iter_;
double integration_dt_;
double backup_Tmax_;

double highest_lfh_L_ = 0;
double highest_lgh_L_ = 0;

// ECOS global variables
bool ecos = true;
int counter = 0;
int counter_ecos = 0;




CyberTimer<1000> filterTimer;

#include "segway_sim/asif_filter.hpp"

using namespace Eigen;






void filterInput(void)
{
	double xNow[nx] = {stateCurrent_.stateVec[0]-0.2,stateCurrent_.stateVec[3],stateCurrent_.stateVec[5],stateCurrent_.stateVec[6]};
	double uDesNow[nu] = {(inputDes_.inputVec[0]+inputDes_.inputVec[1])/2};
	double uActNow[nu] = {0.0};
	double tNow = 0.0;
	double relax[2];

	filterTimer.tic();
	int32_t rc = asif->filter(xNow,uDesNow,uActNow,relax);
	double uQP = uActNow[0]; 
	filterTimer.toc();

	// if (rc < 0) {
	// 	counter++;
	// 	ROS_INFO("QP FAILED %i", counter);
	// }

	// store stuff for Lipschitz constants
	for (int i = 0; i < npBTSS; i++)
	{
		double h_out[npSS];
		double Dh_out[npSS*nx];
		double f_out[nx];
		double g_out[nx*nu];
		safetySet( (*asif).backTraj_[i].second.data(), h_out, Dh_out);
		dynamics( (*asif).backTraj_[i].second.data(), f_out, g_out);
		for (int j = 0; j < nx; j++)
		{
			filter_info_.traj_full[i*nx+j] = (*asif).backTraj_[i].second[j];
			filter_info_.f_full[i*nx+j] = f_out[j];
			filter_info_.g_full[i*nx+j] = g_out[j];
		}

		for (int j = 0; j < nx; j++)
		{
			filter_info_.f_full[i*nx+j] = (*asif).f_out_[i][j];
			for (int k = 0; k < nu; k++)
			{
				filter_info_.g_full[j*nu+i*nx*nu+k] = (*asif).g_out_[i][j][k];
			}
		}
	}

	double h_out[npBS];
	double Dh_out[npBS*nx];
	backupSet( (*asif).backTraj_[npBTSS-1].second.data(), h_out, Dh_out);
	filter_info_.h_full[npSS*npBTSS] = h_out[0];



	// init_and_solve_ecos
	//...........................................................................
	//assume npSS and npBS = 1
	idxint n = 3 + 2;							// Number of decision variables
	idxint m = 3+2*(npBTSS+1) + 4;				// Number of  constraints
	idxint p = 0;								// Number of equality constraints
	idxint l = 4;								// Number of positive orthant constraints 
	idxint ncones = 1+(npBTSS+1);				// Number of second order cones
	idxint q[1+(npBTSS+1)];      				// Array of the length of each of the ncones
	q[0] = 3;									
	for (int i = 1; i < 1+(npBTSS+1); i++)
		q[i] = 2;
	idxint e = 0;								// Number of exponential cones
	idxint *Ajc = NULL;
	idxint *Air = NULL;
	idxint ecos_flag;
	if (!run_once)
	{
		Gpr = new pfloat[3+3*(npBTSS+1)+4];
		c = new pfloat[3 + 4];
		h_ecos = new pfloat[3+2*(npBTSS+1)+4];
		run_once = true;
	}


	double epsilon = 0.0;
	double slack_cost = 100; 
	double alpha_high = 20; 
	double alpha_low = .25;
	double alphaB_low = 5; 
	double L_ah[2] = {1.0, 1.6724};
	double L_Lfh[2] = {2.664, 0.42};
	double L_Lgh[npBTSS+1]; 
	for(int i=0; i<npBTSS; i++){
		L_Lgh[i] = integration_dt_*0.0081;
	}	
	L_Lgh[npBTSS] = 0.0255; 

	// double L_Lgh[npBTSS+1] = {0*0.0081, 0.33*0.0081, 0.66*0.0081, 1*0.0081, 0.0255};

	// Set Inequality Constraints
	idxint Gjc[n+1]; 
	Gjc[0] = 0; 
	Gjc[1] = 2; 
	Gjc[2] = 2; 
	Gjc[3] = 2+(1+2*(npBTSS+1));
	Gjc[4] = 2+(1+2*(npBTSS+1)) + 2 + npBTSS; 
	Gjc[5] = 2+(1+2*(npBTSS+1)) + 2 + npBTSS + 3;

	idxint Gir[3+3*(npBTSS+1) + 4];
	for (int i = 0; i < (int) 3+2*(npBTSS+1); i++)
	{
		Gir[i] = i+;
	}
	Gir[3 + 2*(npBTSS+1)] = 0; 
	Gir[3 + 2*(npBTSS+1)+1] = 1; 
	for (int i=0; i<npBTSS; i++){
		Gir[3 + 2*(npBTSS+1)+2+i] = 7+i*2;
	}
	Gir[3 + 2*(npBTSS+1) + 2 + npBTSS] = 2; 
	Gir[3 + 2*(npBTSS+1) + 2 + npBTSS +1] = 3; 
	Gir[3 + 2*(npBTSS+1) + 2 + npBTSS +2] = 7 + 2*npBTSS; 


	// for (int i = (int) 0; i < (int) (npBTSS+1); i++)
	// {
	// 	Gir[3*i+3+2*(npBTSS+1)] = 2*i; 
	// 	Gir[3*i+4+2*(npBTSS+1)] = 2*i+1;
	//  	Gir[3*i+5+2*(npBTSS+1)] = 3+2*(npBTSS+1) + 2*i;
 // 	}



	Gpr[0] = -1/sqrt(2);
	Gpr[1] = -1/sqrt(2);
	Gpr[2] = -1;

	h_ecos[0] = alpha_high;
	h_ecos[1] = -alpha_low;
	h_ecos[2] = alpha_high; 
	h_ecos[3] = -alphaB_low;
	h_ecos[4] = 1/sqrt(2);
	h_ecos[5] = -1/sqrt(2);
	h_ecos[6] = 0;


	for (int i = 0; i < (int) npBTSS+1; i++)
	{
		Gpr[3 + i*2] = -asif->Lgh_out_[i][0];
		Gpr[4 + i*2] = -epsilon*L_Lgh[i];
		if (i<npBTSS)
		{
			h_ecos[7 + i*2] = (*asif).Lfh_out_[i]  - epsilon*L_Lfh[0] ;
		}else{
			h_ecos[7 + i*2 +1] = (*asif).Lfh_out_[i]  - epsilon*L_Lfh[1] ;
		}
	}

	Gpr[3 + 2*(npBTSS+1)] = 1; 
	GPr[3 + 2*(npBTSS+1) + 1] = -1;

	for (int i = 0; i<(int) npBTSS; i++){
		GPr[3 + 2*(npBTSS+1) + 2 + i] = -(*asif).h_out_[i]+epsilon*L_ah[0];
	}

	Gpr[3 + 2*(npBTSS+1) + 2 + npBTSS] = 1; 
	GPr[3 + 2*(npBTSS+1) + 2 + npBTSS + 1] = -1;
	GPr[3 + 2*(npBTSS+1) + 2 + npBTSS + 2] = -(*asif).h_out_[i]+epsilon*L_ah[0];


	// for (int i = 0; i < (int) npBTSS+1; i++)
	// {
	// 	Gpr[3 + 2*(npBTSS+1) + i*3] = 1;
	// 	Gpr[4 + 2*(npBTSS+1) + i*3] = -1;
	// 	if (i<npBTSS){
	// 		Gpr[5 + 2*(npBTSS+1) + i*3] = -(*asif).h_out_[i]+epsilon*L_ah[0];
	// 	}else{
	// 		Gpr[5 + 2*(npBTSS+1) + i*3] = -(*asif).h_out_[i]+epsilon*L_ah[1];
	// 	}
	// 	h_ecos[2*i] = alpha_high; 
	// 	h_ecos[2*i+1] = -alpha_low;
	// }
	// h_ecos[2*npBTSS+1] = -alphaB_low;
	
	// Set Linear Cost
	c[0] = 1;
	c[1] = 0;
	c[2] = -uDesNow[0];
	c[3] = slack_cost; 
	c[4] = slack_cost;
	// for (int i=0; i<(npBTSS+1); i++){
	// 	c[i+3] = slack_cost; 
	// }

	// Define and Solve SOCP
	problem = ECOS_setup(n, m, p, l, ncones, q, e, Gpr, Gjc, Gir, Apr, Ajc, Air, c, h_ecos, b);
	ecos_flag = ECOS_solve(problem);
	ECOS_cleanup(problem,0);


	for (int i = 0; i < npBTSS*npSS+npBS; i++)
	{
		filter_info_.lfh_full[i] = (*asif).Lfh_out_[i];
		filter_info_.h_full[i] = (*asif).h_out_[i];
		for (int j = 0; j < nu; j++)
			filter_info_.lgh_full[i*nu+j] = (*asif).Lgh_out_[i][j];
	}


	filter_info_.hSafetyNow = asif->hSafetyNow_;
	filter_info_.indexDebug = asif->index_debug_;
	for (int i = 0; i < npSS; i++) {
		filter_info_.hDebug[i] = asif->h_index_[i];
	}

	filter_info_.hBackupEnd = asif->hBackupEnd_;
	filter_info_.filterTimerUs = filterTimer.getAverage()*1.0e6;

	// filter_info_.BTorthoBS = asif->BTorthoBS_;
	// filter_info_.TTS = asif->TTS_;
	for (int i = 0; i < nx*npSS; i++)
		filter_info_.DhDebug[i] = asif->Dh_index_[i];
	filter_info_.asifStatus = ASIF::ASIFimplicit::filterErrorMsgString(rc);
	filter_info_.relax1 = relax[0];
	filter_info_.relax2 = relax[1];

	std::copy((*asif).backTraj_.back().second.begin(),(*asif).backTraj_.back().second.begin()+4,filter_info_.xBackupEnd.begin());

	double u_diff = 0;//(inputDes_.inputVec[0]-inputDes_.inputVec[1]);

	if(passThrough_>0 /*|| rc==-1 || rc==2*/)
	{
		std::copy(inputDes_.inputVec.begin(),inputDes_.inputVec.end(),inputAct_.inputVec.begin());
	}
	else
	{
		if (ecos_flag==0 or ecos_flag==10){
			inputAct_.inputVec[0] = problem->x[2]+u_diff/2;
			inputAct_.inputVec[1] = problem->x[2]-u_diff/2;
			//ROS_INFO("Slack: %f", problem->x[3]);
		}else{
			inputAct_.inputVec[0] = uActNow[0]+u_diff/2;
			inputAct_.inputVec[1] = uActNow[0]-u_diff/2;
			counter_ecos++;
			ROS_INFO("ECOS FAILED %i", counter_ecos);
		}
		//ROS_INFO("input: %f, \t %f", uQP, problem->x[2]);
		//ROS_INFO("alpha: %f, \t %f", problem->x[3], relax[0]);

	}

	iter_++;
	inputAct_.status = static_cast<uint8_t>(STATUS::RUNNING);
	inputAct_.header.seq = iter_;
	inputAct_.header.stamp = ros::Time::now();
	inputAct_.header.frame_id = std::string("stateSeq=") + std::to_string(stateCurrent_.header.seq) + std::string(", inputDesSeq=") + std::to_string(inputDes_.header.seq);
	backTrajMsg_.header.seq = iter_;
	backTrajMsg_.header.stamp = inputAct_.header.stamp;
	filter_info_.header.seq = iter_;
	filter_info_.header.stamp = inputAct_.header.stamp;
}

void inputCallback(const segway_sim::input::ConstPtr msg)
{
	inputDes_ = *msg;
	filterInput();
	pub_inputAct_.publish(inputAct_);
	pub_info_.publish(filter_info_);

	Quaterniond cyberpod_q;
	Vector3d cyberpod_eul;
	cyberpod_eul(0) = 0.0;
	cyberpod_eul(2) = 0.0;
	uint32_t i = 0;
	for(auto & pose : backTrajMsg_.poses)
	{
		cyberpod_eul(1) = (*asif).backTraj_[i].second[2];
		eul2quatZYX(cyberpod_eul,cyberpod_q);

		pose.pose.position.x = (*asif).backTraj_[i].second[0];
		pose.pose.orientation.w = cyberpod_q.w();
		pose.pose.orientation.x = cyberpod_q.x();
		pose.pose.orientation.y = cyberpod_q.y();
		pose.pose.orientation.z = cyberpod_q.z();

		i++;
	}

	pub_backupTraj_.publish(backTrajMsg_);
}

void stateCallback(const segway_sim::state::ConstPtr msg)
{

	stateCurrent_ = *msg;
}


int main(int argc, char *argv[])
{

	// Init ros
	ros::init(argc,argv,"safety_filter");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<segway_sim::state>("state_true", 1, stateCallback);
	sub_inputDes_ = nh_->subscribe<segway_sim::input>("inputDes", 1, inputCallback);

	pub_inputAct_ = nh_->advertise<segway_sim::input>("input", 1);
	pub_info_ = nh_->advertise<segway_sim::filterInfo>("safety_filter_info", 1);
	pub_backupTraj_ = nh_->advertise<nav_msgs::Path>("backup_traj", 1);

	// Retreive params
	nhParams_->param<int32_t>("pass_through",passThrough_,0);
	nhParams_->param<double>("integration_dt",integration_dt_,0.01);
	nhParams_->param<double>("backup_Tmax",backup_Tmax_,1.0);

	if(passThrough_!=0 && passThrough_!=1)
	{
		passThrough_ = 1;
		ROS_WARN("passTrough must be 0 or 1. Will be set to %i",passThrough_);
	}

	// Initialize variables
	iter_ = 0;
	backTrajMsg_.header.frame_id = "/world";
	inputDes_.inputVec.fill(0.0);

	// Initialize ASIF

	asif = new ASIF::ASIFimplicit(nx,nu,npSS,npBS,npBTSS,
	                              safetySet,backupSet,dynamics,dynamicsGradients,backupController);

	ASIF::ASIFimplicit::Options opts;
	opts.backTrajHorizon = backup_Tmax_;
	opts.backTrajDt = integration_dt_;
	opts.relaxReachLb = 5.;
	opts.relaxSafeLb = .25;
	opts.relaxCost = 5*5000;
	opts.n_debug = -1;

	asif->initialize(lb,ub,opts);

	for (int i = 0; i < npBTSS*npSS+npBS; i++)
		filter_info_.h_full.push_back(0.0);

	for (int i = 0; i < nx*(npBTSS*npSS+npBS); i++)
		filter_info_.Dh_full.push_back(0.0);

	for (int i = 0; i < npBTSS*nx; i++)
		filter_info_.traj_full.push_back(0.0);

	for (int i = 0; i < npBTSS*npSS+npBS; i++)
		filter_info_.lfh_full.push_back(0.0);

	for (int i = 0; i < nu*(npBTSS*npSS+npBS); i++)
		filter_info_.lgh_full.push_back(0.0);

	for (int i = 0; i < nx*(npBTSS*npSS+npBS); i++)
		filter_info_.f_full.push_back(0.0);

	for (int i = 0; i < nx*nu*(npBTSS*npSS+npBS); i++)
		filter_info_.g_full.push_back(0.0);

	for (int i = 0; i < nx*npSS; i++)
		filter_info_.DhDebug.push_back(0.0);

	for (int i = 0; i < npSS; i++)
		filter_info_.hDebug.push_back(0.0);

	for (int i = 0; i < nu; i++)
		filter_info_.Lgh_diff.push_back(0.0);

	filter_info_.Lfh_diff.push_back(0.0);

	geometry_msgs::PoseStamped poseTmp;
	poseTmp.header.frame_id = "/world";
	poseTmp.pose.position.z = 0.195;
	poseTmp.pose.position.y = 0.0;
	backTrajMsg_.poses.resize((*asif).backTraj_.size(),poseTmp);

	// Display node info
	ROS_INFO("Safety Filter node successfuly started with:");
	ROS_INFO("___pass_through=%u",passThrough_);
	ROS_INFO("___integration_dt=%.3f",integration_dt_);
	ROS_INFO("___backup_Tmax=%.3f",backup_Tmax_);

	// Take it for a spin
	ros::spin();

	return 0;
}
