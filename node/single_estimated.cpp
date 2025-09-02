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
#include <sensor_msgs/Imu.h>
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/Plot.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

struct MAV_eigen
{
    Eigen::Vector3d r;        // position
    Eigen::Vector3d r_c;      // camera position
    Eigen::Vector3d v;        // velocity
    Eigen::Vector3d a_imu;    // acceleration from IMU
    Eigen::Vector3d omega_c;  // angular velocity command
    Eigen::Matrix3d R_w2b;    // rotation matrix from world to body frame
    Eigen::Quaterniond q;     // quaternion orientation
};
struct EIF_data
{
    Eigen::VectorXd X; //state
	Eigen::VectorXd X_hat;
	Eigen::VectorXd xi;
	Eigen::VectorXd xi_hat;
	Eigen::VectorXd y;
    Eigen::VectorXd y_alt;
	Eigen::VectorXd h;
    Eigen::VectorXd h_alt;
    Eigen::VectorXd z;
    Eigen::VectorXd pre_z;
    Eigen::VectorXd z_alt;
    Eigen::VectorXd pre_z_alt;

    Eigen::MatrixXd s;
    Eigen::MatrixXd s_alt;
	Eigen::MatrixXd F;
	Eigen::MatrixXd H;
    Eigen::MatrixXd H_alt;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_hat;
    Eigen::MatrixXd Omega;
    Eigen::MatrixXd Omega_hat;
    
    std::vector<Eigen::MatrixXd> passive_s;
    std::vector<Eigen::VectorXd> passive_y;
    std::vector<int> passive_id;
    int ID;
};

MAV_eigen mav_eigen_self;
EIF_data self;

Eigen::MatrixXd Q; //noise matrix
Eigen::MatrixXd R; //noise matrix
Eigen::MatrixXd R_alt; //vision measurement noise matrix
Eigen::VectorXd u; //input

bool init, pose_init;
Eigen::Vector3d gt;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!init) init = true;
    // std::cout << "Imu received\n";
    mav_eigen_self.a_imu(0) = msg->linear_acceleration.x;
    mav_eigen_self.a_imu(1) = msg->linear_acceleration.y;
    mav_eigen_self.a_imu(2) = msg->linear_acceleration.z;

    mav_eigen_self.omega_c(0) = msg->angular_velocity.x;
    mav_eigen_self.omega_c(1) = msg->angular_velocity.y;
    mav_eigen_self.omega_c(2) = msg->angular_velocity.z;

    mav_eigen_self.R_w2b = Eigen::Quaterniond(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    ).toRotationMatrix().inverse();

}
void measurement_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    self.z(0) = msg->pose.position.x;
    self.z(1) = msg->pose.position.y;
}
void alt_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    self.z_alt(0) = msg->pose.position.z;

    // std::cout << "Pose received\n";
}
void gt_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!pose_init) {
        pose_init = true;
        self.X(0) = msg->pose.position.x;
        self.X(1) = msg->pose.position.y;
        self.X(2) = msg->pose.position.z;
    }
    // std::cout << "GT received\n";
    gt(0) = msg->pose.position.x;
    gt(1) = msg->pose.position.y;
    gt(2) = msg->pose.position.z;
    std::cout << "GT: " << gt(0) << ", " << gt(1) << ", " << gt(2) << "\n";
}

void process(double delta_t){

    double dt = static_cast<double>(delta_t);
    // double dt = 0.001;
    Eigen::Vector3d world_a = mav_eigen_self.R_w2b.inverse()*mav_eigen_self.a_imu; 
    world_a(2) += -9.80665;

    self.F.setIdentity();
    self.F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity(3, 3)*dt;
    // u.segment(0, 3) = 1/2*dt*dt*world_a;
    u.segment(0, 3) = self.X.segment(3, 3)* dt; 
    u.segment(3, 3) = world_a*dt;
    self.X_hat = self.F*self.X + u;

    self.P_hat = self.F*self.P*self.F.transpose() + Q;

}
void computeCorrPair(Eigen::Vector2d z, Eigen::VectorXd z_alt){

    self.z = z;

    self.s.setZero();
    self.y.setZero();
    self.s_alt.setZero();
    self.y_alt.setZero();

    if(self.pre_z_alt != z_alt)
    {
        self.h_alt(0) = self.X_hat(2);
        self.H_alt(0, 2) = 1;

        self.s_alt = self.H_alt.transpose()*R_alt.inverse()*self.H_alt;
        self.y_alt = self.H_alt.transpose()*R_alt.inverse()*(self.z_alt - self.h_alt + self.H_alt*self.X_hat);
        
    }

    if(self.z != self.pre_z)
    {
        std::cout << "Measurement received\n";
        self.h = self.X_hat.segment(0, 2);
        self.H.block(0, 0, 2, 2).setIdentity();
        self.s = self.H.transpose()*R.inverse()*self.H;
        self.y = self.H.transpose()*R.inverse()*(self.z - self.h + self.H*self.X_hat);
    }

    self.P = (self.P_hat.inverse() + self.s + self.s_alt).inverse();
    self.X = self.P*(self.P_hat.inverse()*self.X_hat + (self.y + self.y_alt));
    self.pre_z = self.z;
    self.pre_z_alt = self.z_alt;
}

double rmse(){
    static Eigen::Vector3d rmse_sum;
    double rmse_val;

    rmse_sum = gt - self.X.segment(0, 3);

    rmse_val = rmse_sum.norm();

    return rmse_val;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "single_estimation");
    ros::NodeHandle nh;
    std::string vehicle;
    bool consensus = false;
    bool position_estimation = false; 
    int mavNum = 1;
    int rosRate = 50;
    int ID = 0;
    int state_size = 6;
    double last_t, dt;
    init = false;
    pose_init = false;
    ros::param::get("mavNum", mavNum);
    ros::param::get("vehicle", vehicle);
    ros::param::get("consensus", consensus);
    ros::param::get("stateSize", state_size);
    ros::param::get("rate", rosRate);
    ros::param::get("pos_est", position_estimation);
    
    ros::Rate rate(rosRate);



    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10, &measurement_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &imu_cb);
    ros::Subscriber alt_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &alt_cb);
    ros::Subscriber gt_sub = nh.subscribe<geometry_msgs::PoseStamped>("/MAV2/mavros/local_position/pose_initialized", 10, &gt_cb);
    ros::Publisher fusedPose_pub = nh.advertise<geometry_msgs::PoseStamped>("singleEst/pose", 10);
    ros::Publisher fusedTwist_pub = nh.advertise<geometry_msgs::TwistStamped>("singleEst/twist", 10);
    ros::Publisher rmse_pub = nh.advertise<std_msgs::Float64MultiArray>("singleEst/rmse", 10);
    self.F.setZero(state_size, state_size);
    self.X.setZero(state_size);
    self.X_hat.setZero(state_size);
    self.P.setZero(state_size, state_size);
    self.P_hat.setZero(state_size, state_size);
    self.z.setZero(2);
    self.pre_z.setZero(2);
    self.z_alt.setZero(1);
    self.pre_z_alt.setZero(1);
    u.setZero(state_size);
    self.P = 1e1*Eigen::MatrixXd::Identity(state_size, state_size);
    


    self.h.setZero(2);
    self.h_alt.setZero(1);
    self.H.setZero(2, state_size);
    self.H_alt.setZero(1, state_size);
    self.s.setZero(state_size, state_size);
    self.y.setZero(state_size);
    self.s_alt.setZero(state_size, state_size);
    self.y_alt.setZero(state_size);


    Q = 1e-3*Eigen::MatrixXd::Identity(6, 6); // process noise
    // Q.block(0, 0, 3, 3) = 1e-4*Eigen::MatrixXd::Identity(3, 3); // position
    Q.block(3, 3, 3, 3) = 8e-2*Eigen::MatrixXd::Identity(3, 3); // velocity
    R = 4e2*Eigen::MatrixXd::Identity(2, 2); // measurement noise
    R_alt = 1*Eigen::MatrixXd::Identity(1, 1); // measurement noise
    self.P = Eigen::MatrixXd::Identity(state_size, state_size);
    self.P_hat = Eigen::MatrixXd::Identity(state_size, state_size);

    while(ros::ok() && !init ){
        ROS_INFO("Waiting for FCU data...");
        ros::spinOnce();
        rate.sleep();
    }
    
    while(ros::ok() && !pose_init ){
        ROS_INFO("Waiting for Pose data...");
        ros::spinOnce();
        rate.sleep();
    }


    
    last_t = ros::Time::now().toSec();
    std::cout << "Initialization done\n";

    while(ros::ok()){

        process(ros::Time::now().toSec() - last_t);
        computeCorrPair(self.z, self.z_alt);
        
        
        
        last_t = ros::Time::now().toSec();
        std::cout << "Estimated position: " << self.X(0) << ", " << self.X(1) << ", " << self.X(2) << "\n";
        std::cout << "RMSE : "<< rmse() <<"\n";

        geometry_msgs::PoseStamped fusedPoseMsg;
        geometry_msgs::TwistStamped fusedTwistMsg;
        fusedPoseMsg.header.stamp = ros::Time::now();
        fusedPoseMsg.pose.position.x = self.X(0);
        fusedPoseMsg.pose.position.y = self.X(1);
        fusedPoseMsg.pose.position.z = self.X(2);

        fusedTwistMsg.header.stamp = ros::Time::now();
        fusedTwistMsg.twist.linear.x = self.X(3);
        fusedTwistMsg.twist.linear.y = self.X(4);
        fusedTwistMsg.twist.linear.z = self.X(5);
        
        std_msgs::Float64MultiArray rmseMsg;
        rmseMsg.data.push_back(rmse());
        
        rmse_pub.publish(rmseMsg);
        fusedPose_pub.publish(fusedPoseMsg);
        fusedTwist_pub.publish(fusedTwistMsg);


        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}