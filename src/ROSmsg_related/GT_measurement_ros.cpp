#include "GT_measurement_ros.h"

GT_measurement::GT_measurement(ros::NodeHandle& nh_, int id, int mavnum)
{
        nh = nh_;
        ID = id;
        self_index = ID;
        mavNum = mavnum;
        formation_num = mavNum;

        /*=================================================================================================================================
            groundtruth
        =================================================================================================================================*/
        GTs_rate = 500;
        GTs_count = 0;
        GTs = new MAV[mavNum];
        
        // Subscribe to /MAVx/mavros/local_position/pose_initialized for each MAV
        groundTruth_subs.resize(mavNum);
        std::vector<std::string> topics = {"MAV1", "MAV2", "MAV6"}; // Topic names for IDs 1, 2, 3
        for (int i = 0; i < mavNum; i++) {
            std::string topic = "/" + topics[i] + "/mavros/local_position/pose_initialized";
            groundTruth_subs[i] = nh.subscribe<geometry_msgs::PoseStamped>(
                topic, 30, boost::bind(&GT_measurement::groundTruth_cb, this, _1, i));
        }

        /*=================================================================================================================================
            Lidar, position
        ===============================================================================================================================*/
        lidar_rate = 100;
        position_rate = 50;

        /*=================================================================================================================================
            map matching
        =================================================================================================================================*/
        map_sub = nh.subscribe<geometry_msgs::PoseStamped>("vsnav_pose", 2, &GT_measurement::map_callback, this);
}
// GT_measurement::GT_measurement(ros::NodeHandle& nh_, int id, int mavnum)
// {
// 	nh = nh_;
// 	ID = id;
// 	cout << mavnum << " mavNum\n";
// 	GTs = new MAV[mavNum];
// 	GTs_eigen.resize(mavNum);
// 	/*=================================================================================================================================
// 		groundtruth
// 	=================================================================================================================================*/
//   	groundTruth_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 30, &GT_measurement::pose_cb, this);
// 	map_sub = nh.subscribe<geometry_msgs::PoseStamped>("vsnav_pose", 2, &GT_measurement::map_callback, this);

// }
GT_measurement::~GT_measurement()
{
    delete[] GTs;
}

void GT_measurement::setRosRate(int rate)
{
	rosRate = rate;
}
//=================================================================================================================================
//   groundtruth exp	
//=================================================================================================================================
void GT_measurement::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	GTs_count++; // GroundTruth call back rate = 500hz
	// std::cout << "GTs_count: " << msg->pose.position.x << "\n";
	////////////////////////// get groundTruth model states and arrange their ID////////////////////
	GTs[ID].setPose(msg->pose);
	GTs[ID].setTwist(geometry_msgs::Twist());
	MAV_eigen Mav_eigen;
    // std::cout << Mav.getPose().pose << std::endl;
	Mav_eigen.r(0) = GTs[ID].getPose().pose.position.x;
	Mav_eigen.r(1) = GTs[ID].getPose().pose.position.y;
	Mav_eigen.r(2) = GTs[ID].getPose().pose.position.z;
	Mav_eigen.v(0) = GTs[ID].getVel().twist.linear.x;
	Mav_eigen.v(1) = GTs[ID].getVel().twist.linear.y;
	Mav_eigen.v(2) = GTs[ID].getVel().twist.linear.z;
	Mav_eigen.a_imu(0) = GTs[ID].getAcc().x;
	Mav_eigen.a_imu(1) = GTs[ID].getAcc().y;
	Mav_eigen.a_imu(2) = GTs[ID].getAcc().z;
	
	Mav_eigen.omega_c(0) = GTs[ID].getVel().twist.angular.x;
	Mav_eigen.omega_c(1) = GTs[ID].getVel().twist.angular.y;
	Mav_eigen.omega_c(2) = GTs[ID].getVel().twist.angular.z;
	Mav_eigen.R_w2b = Eigen::Quaterniond(
		GTs[ID].getPose().pose.orientation.w,
		GTs[ID].getPose().pose.orientation.x,
		GTs[ID].getPose().pose.orientation.y,
		GTs[ID].getPose().pose.orientation.z
	).toRotationMatrix().inverse();
	Mav_eigen.q.w() = GTs[ID].getPose().pose.orientation.w;
	Mav_eigen.q.x() = GTs[ID].getPose().pose.orientation.x;
	Mav_eigen.q.y() = GTs[ID].getPose().pose.orientation.y;
	Mav_eigen.q.z() = GTs[ID].getPose().pose.orientation.z;
	GTs_eigen[ID] = Mav_eigen;

	static std::default_random_engine generator;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);//uniform distribution between 0 and 1
	// std::cout << dis(gen) << "\n ";
	// if(GTs_count % (GTs_rate/lidar_rate) == 0) // lidar_rate = 10hz means that we do a measurement evry 50 count 
	double a = dis(gen);
	if (a < 0.01)
	{
		altitude_measure = GTs_eigen[ID].r(2)  ; // altitude measure
	}

	// GTs_eigen = mavsMsg2Eigen(GTs, mavNum);
	// std::cout<< GTs_eigen.size() <<"test\n";
}

/*=================================================================================================================================
    groundtruth
=================================================================================================================================*/

void GT_measurement::groundTruth_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, int mav_index)
{
        // ROS_INFO("Received data for MAV%d (mav_index: %d)", mav_index + 1, mav_index); // Debug: Confirm callback is triggered
        GTs_count++; // Increment callback counter

        // Store pose for the specific MAV
        GTs[mav_index].setPose(msg->pose); // Assumes setPose accepts geometry_msgs::Pose
        // ROS_INFO("Stored pose for MAV%d: position (%f, %f, %f)", 
                //  mav_index + 1, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
        // Set twist to zero (PoseStamped does not provide twist)
        geometry_msgs::Twist zero_twist;
        zero_twist.linear.x = 0.0;
        zero_twist.linear.y = 0.0;
        zero_twist.linear.z = 0.0;
        zero_twist.angular.x = 0.0;
        zero_twist.angular.y = 0.0;
        zero_twist.angular.z = 0.0;
        GTs[mav_index].setTwist(zero_twist);

        // Track which MAVs have reported
        static std::vector<bool> received(mavNum, false);
        received[mav_index] = true;

        // Check if all MAVs have reported
        bool all_received = true;
        for (size_t i = 0; i < received.size(); i++) {
            if (!received[i]) {
                ROS_WARN("MAV%d has not reported yet", i + 1);
                all_received = false;
            }
        }

        if (all_received) {
            // ROS_INFO("All MAVs reported, updating GTs_eigen");
            GTs_eigen = mavsMsg2Eigen(GTs, mavNum);
            // Debug: Log GTs_eigen contents
            for (size_t i = 0; i < GTs_eigen.size(); i++) {
                // ROS_INFO("GTs_eigen[%zu] updated for MAV%d", i, i + 1);
            }
            std::vector<MAV_eigen> formation_eigen_GT(GTs_eigen.begin() + 1, GTs_eigen.end()); // First one is target

            // Transform from groundtruth to measurements
            static std::default_random_engine generator;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);

            double a = dis(gen);
            if (a < 0.01) {
                // ROS_INFO("Performing measurements for all MAVs");
                lidarMeasurements = lidarMeasure(formation_eigen_GT, generator);
                lidar4target = lidarmeasure4target(formation_eigen_GT, GTs_eigen[0], generator);
                CameraModel = Camera4Neighbor(formation_eigen_GT, generator);
                CameraModel4target = CameraMeasure4target(formation_eigen_GT, GTs_eigen[0], generator);
                // pinhole_model(formation_eigen_GT, generator);
            }
            if (GTs_count % (GTs_rate / position_rate) == 0) {
                positionMeasurement = positionMeasure(GTs_eigen[ID], generator);
            }
            if (GTs_count == GTs_rate) {
                GTs_count = 0;
            }
        } else {
            ROS_WARN("Not all MAVs reported, skipping GTs_eigen update");
        }
}

std::vector<MAV_eigen> GT_measurement::getGTs_eigen(){	return GTs_eigen;}
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
			measurement(3) = i; // ID

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

/*=================================================================================================================================
	map matching
=================================================================================================================================*/
void GT_measurement::map_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// if (abs(GTs_eigen[ID].r(0) - msg->pose.position.x)  < 10 && abs(GTs_eigen[ID].r(1) - msg->pose.position.y) < 10)
	// {
	std::cout << "map matching: " << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << "\n";
		map_measure(0) = msg->pose.position.x;
		map_measure(1) = msg->pose.position.y;
		map_measure(2) = 55;//msg->pose.position.z;
	// }
		// map_uv(0) = msg->pose.orientation.z;
		// map_uv(1) = msg->pose.orientation.w;
}
Eigen::Vector3d GT_measurement::getMapMeasure(){return map_measure;}
// Eigen::Vector3d GT_measurement::getAltitudeMeasure(){return altitude_measure;}
Eigen::Vector2d GT_measurement::getuv(){return map_uv;}

