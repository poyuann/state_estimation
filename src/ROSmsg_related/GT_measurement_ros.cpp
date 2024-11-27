#include "GT_measurement_ros.h"

GT_measurement::GT_measurement(ros::NodeHandle& nh_, int id, int mavnum)
{
    nh = nh_;
    ID = id;
    self_index = ID-1;
    mavNum = mavnum;
    formation_num = mavNum-1;

	/*=================================================================================================================================
		groundtruth
	=================================================================================================================================*/
  	groundTruth_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 30, &GT_measurement::groundTruth_cb, this);
	GTs_rate = 500;
	GTs_count = 0;
	GTs = new MAV[mavNum];

	/*=================================================================================================================================
        Lidar, position
    ===============================================================================================================================*/
	lidar_rate = 100;
	position_rate = 50;

	/*=================================================================================================================================
        Camera boundingBox
    =================================================================================================================================*/	
    bboxes_sub = nh.subscribe<std_msgs::Float64MultiArray>("synchronizer/yolov8/boundingBox", 2, &GT_measurement::bboxes_cb, this);
	bbox_count = 0;
	no_bbox_count = 0;
	checkCount = 0;
	bbox_eigen_past << 320, 240, 4;
	bbox_eigen = bbox_eigen_past;
	gotBbox  = false;
}

GT_measurement::~GT_measurement()
{
    delete[] GTs;
}

void GT_measurement::setRosRate(int rate)
{
	rosRate = rate;
}

/*=================================================================================================================================
    groundtruth
=================================================================================================================================*/

void GT_measurement::groundTruth_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	GTs_count++; // GroundTruth call back rate = 500hz

	////////////////////////// get groundTruth model states and arrange their ID////////////////////
	std::vector<string> name = msg->name;
	for(int i=0; i< name.size(); i++)
	{
		if(std::isdigit(name[i].back())) ////// First one is ground, skip it
		{
			GTs[int(name[i].back()-'0')].setPose(msg->pose[i]);
			GTs[int(name[i].back()-'0')].setTwist(msg->twist[i]);
		}
	}
	GTs_eigen = mavsMsg2Eigen(GTs, mavNum);
	std::vector<MAV_eigen> formation_eigen_GT(GTs_eigen.begin()+1, GTs_eigen.begin()+GTs_eigen.size()); // First one is target, we want all UAV

	////////////////////////// Transform from groundtruth to measurements,  ////////////////////////
	static std::default_random_engine generator;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);//uniform distribution between 0 and 1
	// std::cout << dis(gen) << "\n ";
	// if(GTs_count % (GTs_rate/lidar_rate) == 0) // lidar_rate = 10hz means that we do a measurement evry 50 count 
	double a = dis(gen);
	if (a < 0.01)
	{
		// std::cout <<a<<"haha\n\n";
		lidarMeasurements = lidarMeasure(formation_eigen_GT, generator);
		lidar4target = lidarmeasure4target(formation_eigen_GT,GTs_eigen[0], generator);
		CameraModel = Camera4Neighbor(formation_eigen_GT, generator);
		CameraModel4target = CameraMeasure4target(formation_eigen_GT,GTs_eigen[0], generator);
		pinhole_model(formation_eigen_GT, generator);
	}
	if(GTs_count % (GTs_rate/position_rate) == 0) // position_rate = 10hz means that we do a measurement evry 50 count 
		positionMeasurement = positionMeasure(GTs_eigen[ID], generator);
	if(GTs_count == GTs_rate)
		GTs_count = 0;
}

std::vector<MAV_eigen> GT_measurement::getGTs_eigen(){return GTs_eigen;}
geometry_msgs::Quaternion GT_measurement::getGTorientation(int ID){return GTs[ID].getPose().pose.orientation;}


/*=================================================================================================================================
    Lidar, position
===============================================================================================================================*/

std::vector<Eigen::Vector4d> GT_measurement::lidarMeasure(std::vector<MAV_eigen> formation_GT, std::default_random_engine generator)
{
	Eigen::Vector4d measurement;
	std::vector<Eigen::Vector4d> measurements;
	Eigen::Vector3d r_ns_B;
	Eigen::Matrix3d R_W2B = formation_GT[self_index].R_w2b;
	for(int i=0; i<formation_num; i++)
	{
		if(i != self_index)
		{
			r_ns_B = R_W2B*(formation_GT[i].r - formation_GT[self_index].r);

			measurement(0) = sqrt(pow(r_ns_B(0), 2) + pow(r_ns_B(1), 2) + pow(r_ns_B(2), 2));
			measurement(1) = acos(r_ns_B(2)/measurement(0)); // theta
			measurement(2) = atan2(r_ns_B(1), r_ns_B(0)); // phi
			measurement(3) = i+1; // ID

			std::normal_distribution<double> n_D(0.0, 0.02);
			std::normal_distribution<double> n_theta(0.0, 0.035);
			std::normal_distribution<double> n_phi(0.0, 0.035);
			measurement(0) += n_D(generator);
			measurement(1) += n_theta(generator);
			measurement(2) += n_phi(generator);

			measurements.push_back(measurement);
		}
	}
	return measurements;
}
Eigen::Vector3d GT_measurement::lidarmeasure4target(std::vector<MAV_eigen> formation_GT,MAV_eigen target_eigen, std::default_random_engine generator)
{
	Eigen::Vector3d measurement;
	Eigen::Vector3d r_ns_B;
	Eigen::Matrix3d R_W2B = formation_GT[self_index].R_w2b;

	r_ns_B = R_W2B*(target_eigen.r - formation_GT[self_index].r);

	measurement(0) = sqrt(pow(r_ns_B(0), 2) + pow(r_ns_B(1), 2) + pow(r_ns_B(2), 2));
	measurement(1) = acos(r_ns_B(2)/measurement(0)); // theta
	measurement(2) = atan2(r_ns_B(1), r_ns_B(0)); // phi

	std::normal_distribution<double> n_D(0.0, 0.02);
	std::normal_distribution<double> n_theta(0.0, 0.035);
	std::normal_distribution<double> n_phi(0.0, 0.035);
	measurement(0) += n_D(generator);
	measurement(1) += n_theta(generator);
	measurement(2) += n_phi(generator);		
		
	return measurement;
}
Eigen::Vector3d GT_measurement::positionMeasure(MAV_eigen GT_eigen, std::default_random_engine generator)
{
	Eigen::Vector3d measurement = GT_eigen.r;

	std::normal_distribution<double> n_x(0.0, 0.05);
	std::normal_distribution<double> n_y(0.0, 0.05);
	std::normal_distribution<double> n_z(0.0, 0.05);
	measurement(0) += n_x(generator);
	measurement(1) += n_y(generator);
	measurement(2) += n_z(generator);

	return measurement;
}

std::vector<Eigen::Vector4d> GT_measurement::getLidarMeasurements(){return lidarMeasurements;}
Eigen::Vector3d GT_measurement::getlidar4target(){return  lidar4target;}
Eigen::Vector3d GT_measurement::getPositionMeasurement(){return positionMeasurement;}
/*=================================================================================================================================
    Camera model
=================================================================================================================================*/

std::vector<Eigen::Vector4d> GT_measurement::Camera4Neighbor(std::vector<MAV_eigen> formation_GT,std::default_random_engine generator)
{	
	double fx = 343.15907310693535;
    double fy = 343.15907310693535;
	double cx = 320.5;
	double cy = 240.5;
	double X,Y,Z;
	Eigen::Vector4d measurement;
	std::vector<Eigen::Vector4d> measurements;

	Eigen::Matrix3d R_b2m ;
	Eigen::Matrix3d R_b2c ;
	Eigen::Matrix3d R_m2p ;
	Eigen::Matrix3d R_p2c ;
	R_b2m << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
	R_p2c << 0, 1, 0,
			0, 0, 1,
			1, 0, 0;
	double yaw, pitch;
	Eigen::Matrix3d R_w2c;// = R_b2c*formation_GT[self_index].R_w2b; ///////////////// rotation problem
	Eigen::Vector3d r_qc_c;
	Eigen::Vector3d q;
	q = formation_GT[self_index].q.toRotationMatrix().eulerAngles(2, 1, 0);
	for(int i=0; i<formation_num; i++)
	{
		if(i != self_index)
		{
			yaw = -atan2(formation_GT[i].r(1) - formation_GT[self_index].r(1),formation_GT[i].r(0) - formation_GT[self_index].r(0));
			yaw += q(0);
			R_m2p << cos(yaw), -sin(yaw), 0,
					sin(yaw), cos(yaw), 0,
					0, 0, 1;
			R_w2c = R_p2c * R_m2p * R_b2m * formation_GT[self_index].R_w2b;
			R_b2c << 0, 1, 0,
				0, 0, 1,
				1, 0, 0;
			// Eigen::Matrix3d R_w2c = R_b2c*formation_GT[self_index].R_w2b; ///////////////// rotation problem
			r_qc_c= R_w2c*(formation_GT[i].r - formation_GT[self_index].r); 
			X = r_qc_c(0)/r_qc_c(2);
			Y = r_qc_c(1)/r_qc_c(2);
			Z = r_qc_c(2);

			measurement(0) = fx*X + cx;
			measurement(1) = fy*Y + cy ;
			measurement(2) = Z;
			measurement(3) = i+1; // ID
			std::normal_distribution<double> n_x(0.0, 1);
			std::normal_distribution<double> n_y(0.0, 1);
			std::normal_distribution<double> n_z(0.0, 0.05);
			measurement(0) += n_x(generator);
			measurement(1) += n_y(generator);
			measurement(2) += n_z(generator);
			measurements.push_back(measurement);
			// std::cout << self_index + 1 <<" to " << i+1 <<"\n"<< measurement.transpose() <<endl ;
		}
	}
	return measurements;

}
Eigen::Vector3d GT_measurement::CameraMeasure4target(std::vector<MAV_eigen> formation_GT,MAV_eigen target_eigen, std::default_random_engine generator)
{	

	Eigen::Vector3d measurement;

	Eigen::Matrix3d R_w2c = cam.R_B2C()*formation_GT[self_index].R_w2b; ///////////////// rotation problem
	Eigen::Vector3d r_qc_c = R_w2c*(target_eigen.r - formation_GT[self_index].r - cam.t_B2C()); 

	double X = r_qc_c(0)/r_qc_c(2);
	double Y = r_qc_c(1)/r_qc_c(2);
	double Z = r_qc_c(2);

	measurement(0) = cam.fx()*X + cam.cx();
	measurement(1) = cam.fy()*Y + cam.cy();
	measurement(2) = Z;
	
	std::normal_distribution<double> n_x(0.0, 5);
	std::normal_distribution<double> n_y(0.0, 5);
	std::normal_distribution<double> n_z(0.0, 0.1);
	measurement(0) += n_x(generator);
	measurement(1) += n_y(generator);
	measurement(2) += n_z(generator);
	return measurement;

}
void GT_measurement::pinhole_model(std::vector<MAV_eigen> formation_GT, std::default_random_engine generator)
{
	Eigen::Matrix3d R_W2B = formation_GT[self_index].R_w2b;
	std::normal_distribution<double> n_u(0.0, 5);
	std::normal_distribution<double> n_v(0.0, 5);
	std::normal_distribution<double> n_d(0.0, 0.10);
	for(int i=0; i<formation_num; i++)
	{
		if(i != self_index)
		{
			Eigen::Vector3d q;
			q = formation_GT[self_index].q.toRotationMatrix().eulerAngles(2, 1, 0);
			if((self_index+formation_num+1)%formation_num == i)
			{
				Eigen::MatrixXd R_w2c = camleft.R_B2C()*R_W2B;
				Eigen::Vector3d r_tc_C = R_w2c*(formation_GT[i].r - formation_GT[self_index].r);
				left_bbox(3) = i+1; // ID
				// left_bbox(0) = camleft.fx()*r_tc_C(0)/r_tc_C(2) + camleft.cx() + n_u(generator);
				// left_bbox(1) = camleft.fy()*r_tc_C(1)/r_tc_C(2) + camleft.cy() + n_v(generator);
				// left_bbox(2) = r_tc_C(2) + n_d(generator);
				left_bbox(0) = camleft.fx()*r_tc_C(0)/r_tc_C(2) + camleft.cx() ;
				left_bbox(1) = camleft.fy()*r_tc_C(1)/r_tc_C(2) + camleft.cy() ;
				left_bbox(2) = r_tc_C(2) ;
				// std::cout << " r_tc_C_l:\n" << left_bbox.transpose() << "\n";
				// std::cout << q(0)<<"\n";
				// std::cout << R_w2c<<"\n";
				// std::cout << -atan2((formation_GT[i].r(1) - formation_GT[self_index].r(1)), (formation_GT[i].r(0) - formation_GT[self_index].r(0)))+q(0)<< "\n";
			}
			if((self_index+formation_num-1)%formation_num == i)
			{
				Eigen::MatrixXd R_w2c = camright.R_B2C()*R_W2B;
				Eigen::Vector3d r_c_W = (formation_GT[self_index].r + R_W2B.inverse()*camright.t_B2C());
				Eigen::Vector3d r_tc_C = R_w2c*(formation_GT[i].r - formation_GT[self_index].r);
				right_bbox(3) = i+1; // ID
				// right_bbox(0) = camright.fx()*r_tc_C(0)/r_tc_C(2) + camright.cx() + n_u(generator);
				// right_bbox(1) = camright.fy()*r_tc_C(1)/r_tc_C(2) + camright.cy() + n_v(generator);
				// right_bbox(2) = r_tc_C(2) + n_d(generator);
				right_bbox(0) = camright.fx()*r_tc_C(0)/r_tc_C(2) + camright.cx() ;
				right_bbox(1) = camright.fy()*r_tc_C(1)/r_tc_C(2) + camright.cy() ;
				right_bbox(2) = r_tc_C(2) ;
				// std::cout << "r_tc_C_r:\n" << right_bbox.transpose() << "\n";
				// std::cout << R_w2c <<"\n";
				// std::cout << -atan2((formation_GT[i].r(1) - formation_GT[self_index].r(1)), (formation_GT[i].r(0) - formation_GT[self_index].r(0)))+q(0)<< "\n";

			}
		}
	}
}
void GT_measurement::setCamera(Camera camera)
{
	cam = camera;
}
void GT_measurement::setNeighborCam(Camera Camleft , Camera Camright)
{
	camleft = Camleft;
	camright = Camright;
}
Eigen::Vector4d GT_measurement::get_left_bbox(){return left_bbox;}
Eigen::Vector4d GT_measurement::get_right_bbox(){return right_bbox;}

std::vector<Eigen::Vector4d>GT_measurement::getCameraNeighbor(){return CameraModel;}
Eigen::Vector3d GT_measurement::getCamera4target(){return  CameraModel4target;}
/*=================================================================================================================================
    Camera boundingBox
=================================================================================================================================*/
void GT_measurement::bboxes_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    bboxes_raw = msg->data;
	
	std::vector<Eigen::Vector3d> bboxes;
	if(bboxes_raw.size() > 3)
	{
		double min_dist = 99999;
		for(size_t i=0; i<bboxes_raw.size(); i+=3)
		{
			Eigen::Vector3d bbox(bboxes_raw[i], bboxes_raw[i+1], bboxes_raw[i+2]);
			bboxes.push_back(bbox);
		}
		for(auto& bbox : bboxes)
		{
			double dist = sqrt(pow(bbox(0) - bbox_eigen_past(0), 2) + pow(bbox(1) - bbox_eigen_past(1), 2));
			if(dist < min_dist)
			{
				min_dist = dist;
				bbox_eigen = bbox;
			}
		}
	}
	else if(bboxes_raw.size() == 3)
			bbox_eigen << bboxes_raw[0], bboxes_raw[1], bboxes_raw[2];
	no_bbox_count = 0;
}

bool GT_measurement::ifCameraMeasure(){return gotBbox;}
void GT_measurement::bbox_check()
{
	if(!gotBbox)
	{
		no_bbox_count = 0;
		checkCount++;
		if(checkCount == rosRate)
		{
			checkCount = 0;
			bbox_count = 0;
			gotBbox = false;
		}
		if(bbox_eigen != bbox_eigen_past)
		{
			bbox_count++;
			if(bbox_count == 10)
			{
				bbox_count = 0;
				checkCount = 0;
				gotBbox = true;
			}
		}
	}
	else
	{
		if(bbox_eigen == bbox_eigen_past)
		{
			no_bbox_count++;
			if(no_bbox_count == rosRate)
				gotBbox = false;
		}
	}
	bbox_eigen_past = bbox_eigen;
}

Eigen::Vector3d GT_measurement::getBboxEigen(){return bbox_eigen;}