#include <string>
#include <sstream>
#include <vector>
#include <cctype>
#include <cmath>
#include <random>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>  
#include <ros/ros.h>
#include "ros/param.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/Plot.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include "Mav.h"
#include "HEIF_self.h"
#include "SEIF_pose.h"
#include "SEIF_neighbors.h"
#include "SEIF_lidar_neighbors.h"
#include "GT_measurement_ros.h"
#include "EIFpairs_ros.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

	// ros::Publisher mavros_fusionPose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
	// ros::Publisher mavros_fusionTwist_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/vision_pose/twist", 10);

	ros::Publisher vo_pub = nh.advertise<geometry_msgs::PoseStamped>("vision_odometry/pose", 10);
	ros::Publisher vo_rmse_pub = nh.advertise<std_msgs::Float64MultiArray>("vision_odometry/rmse", 10);
	
    std::string vehicle;
    bool consensus = false;
	bool position_estimation = false;
	int mavNum = 3;
    int rosRate = 50;
	int ID = 0;
	int state_size = 6;
	double last_t;
	double dt;
	ros::param::get("mavNum", mavNum);
    ros::param::get("vehicle", vehicle);
	ros::param::get("ID", ID);
    ros::param::get("rate", rosRate);
	ros::param::get("consensus", consensus);
	ros::param::get("stateSize", state_size);
	ros::param::get("pos_est", position_estimation);
	
	ros::Rate rate(rosRate);

	geometry_msgs::PoseStamped voMsg;
	geometry_msgs::PoseStamped self_fusedPoseMsg;
	geometry_msgs::TwistStamped self_fusedTwistMsg;
	std_msgs::Float64MultiArray vo_rmseMsg;
	// MAV mav(nh);
    MAV mav(nh, vehicle, ID);
	EIFpairs_ros eif_ros(nh, vehicle, ID, mavNum);
	GT_measurement gt_m(nh, ID, mavNum);
	gt_m.setRosRate(rosRate);
	MAV_eigen mav_eigen;
	
	while(ros::ok())
	{
		std::cout<< mav.imu_init <<"\n";

		if(mav.imu_init)
				break;
		else
			printf("[%s_%i]: Waiting for Imu topic...\n", vehicle.c_str(), ID);
		rate.sleep();
		ros::spinOnce();
	}
	printf("\n[%s_%i EIF]: Topic checked\n", vehicle.c_str(), ID);
	for(int i=0; i< 10; i++)
	{
		std::cout << "Waiting for GT measurement..." << std::endl;
		rate.sleep();
		ros::spinOnce();
	}
	// std::cout << "GT measurement received\n";	
	Self_pose_EIF SEIF_pose;
	Self_rel_EIF SEIF_neighbors;
	Self_lidar_EIF SEIF_lidar_neighbors;
	HEIF_self sheif(6);

	printf("\n[%s_%i EIF]: EIF constructed\n\n", vehicle.c_str(), ID);
	std::cout << "ID: " << gt_m.getGTs_eigen()[ID].r << "\n";
	SEIF_pose.setCurrState(gt_m.getGTs_eigen()[ID]);

	
	dt = 0.001;
	last_t = ros::Time::now().toSec();
	int vml_count = 0;
	// position_estimation = false;
    while(ros::ok())
    {
		vml_count++;
		mav.setOrientation(gt_m.getGTorientation(ID));
		mav_eigen = mavMsg2Eigen(mav);
		/*=================================================================================================================================
			Prediction
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		SEIF_pose.setMavSelfData(mav_eigen);
		SEIF_pose.setMeasurement(gt_m.getVO(), gt_m.getAlt(),gt_m.getMapMeasure());
		SEIF_pose.computePredPairs(dt);
		eif_ros.selfPredEIFpairs_pub.publish(eigen2EifMsg(SEIF_pose.getEIFData(), ID));


		//////////////////     Lidar neighbor  ////////////////////////////////
		SEIF_lidar_neighbors.setMavSelfData(mav_eigen);
		SEIF_lidar_neighbors.setEIFpredData(SEIF_pose.getEIFData());
		SEIF_lidar_neighbors.setLidarMeasurements(gt_m.getLidarMeasurements());
		SEIF_lidar_neighbors.setNeighborData(eif_ros.get_curr_fusing_data(eif_ros.neighborsEIFpairs, 0.1));

		/*=================================================================================================================================
			Correction
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		SEIF_pose.computeCorrPairs();
		SEIF_lidar_neighbors.computeCorrPairs();
		eif_ros.selfPredEIFpairs_pub.publish(eigen2EifMsg(SEIF_lidar_neighbors.getselfEIFData(), ID));

		/*=================================================================================================================================
			Fusion
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		sheif.setSelfEstData(SEIF_pose.getEIFData());
		sheif.setNeighborEstData(SEIF_lidar_neighbors.getEIFData());
		sheif.set_passiveEstData(eif_ros.get_curr_fusing_data(eif_ros.neighborsEIFpairs, 0.05), ID);

		sheif.process();
		SEIF_pose.setFusionPairs(sheif.getFusedCov(), sheif.getFusedState());
		
		std::cout << "SEIF:\n";
		eif_ros.selfState_Plot_pub.publish(compare(gt_m.getGTs_eigen()[ID], sheif.getFusedState() , sheif.getFusedCov(), gt_m.getGTorientation(ID),sheif.getS()));
		
	
		/*=================================================================================================================================
			Publish to mavros for feedback
		=================================================================================================================================*/
		
		// -------------------------------------Position-------------------------------------
		self_fusedPoseMsg.header.frame_id = "/world";
		self_fusedPoseMsg.header.stamp = ros::Time::now();
		self_fusedPoseMsg.pose.position.x = sheif.getFusedState()(0);
		self_fusedPoseMsg.pose.position.y = sheif.getFusedState()(1);
		self_fusedPoseMsg.pose.position.z = sheif.getFusedState()(2);
		self_fusedPoseMsg.pose.orientation.w = mav_eigen.q.w();
		self_fusedPoseMsg.pose.orientation.x = mav_eigen.q.x();
		self_fusedPoseMsg.pose.orientation.y = mav_eigen.q.y();
		self_fusedPoseMsg.pose.orientation.z = mav_eigen.q.z();



		// -------------------------------------Velocity-------------------------------------
		self_fusedTwistMsg.header.stamp = ros::Time::now();
		self_fusedTwistMsg.twist.linear.x = sheif.getFusedState()(3);
		self_fusedTwistMsg.twist.linear.y = sheif.getFusedState()(4);
		self_fusedTwistMsg.twist.linear.z = sheif.getFusedState()(5);
		self_fusedTwistMsg.twist.angular.x = mav_eigen.omega_c(0);
		self_fusedTwistMsg.twist.angular.y = mav_eigen.omega_c(1);
		self_fusedTwistMsg.twist.angular.z = mav_eigen.omega_c(2);

		voMsg.header.frame_id = "/world";
		voMsg.header.stamp = ros::Time::now();
		voMsg.pose.position.x = SEIF_pose.getVO()(0);
		voMsg.pose.position.y = SEIF_pose.getVO()(1);
		voMsg.pose.position.z = SEIF_pose.getVO()(2);

		vo_rmseMsg.data.clear();
		vo_rmseMsg.data.push_back(gt_m.getVO_RMSEmsg(SEIF_pose.getVO()));

		// -------------------------------------debug-----------------------------------------
		// -------------------------------------Publish-------------------------------------
		vo_pub.publish(voMsg);
		vo_rmse_pub.publish(vo_rmseMsg);
		// mavros_fusionPose_pub.publish(self_fusedPoseMsg);
		// mavros_fusionTwist_pub.publish(self_fusedTwistMsg);

		/*=================================================================================================================================
			Descrete time
		=================================================================================================================================*/
		dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();
		rate.sleep();
    	ros::spinOnce();
    }
	
	return 0;
}
