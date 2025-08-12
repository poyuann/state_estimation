#include "SEIF_pose.h"

Self_pose_EIF::Self_pose_EIF()
{
	self_measurement_size = 3;
    EIF_measurement_init(self_state_size, self_measurement_size, &self);
    u.setZero(self_state_size);
    measurement.setZero();
    //////////////////////// Covariance Tuning ////////////////////////

    R = 8e-0*Eigen::MatrixXd::Identity(self_measurement_size, self_measurement_size);
    R(2,2) = 1e-0;
}
Self_pose_EIF::~Self_pose_EIF(){}

void Self_pose_EIF::setMeasurement(Eigen::Vector3d z)
{
    measurement = z;
}
// void Self_pose_EIF::setMapmeasurement(Eigen::Vector2d z)
// {
//     measurement
// }

void Self_pose_EIF::computePredPairs(double delta_t)
{
    double dt = static_cast<double>(delta_t);
    // double dt = 0.001;
    Eigen::Vector3d world_a = Mav_eigen_self.R_w2b.inverse()*Mav_eigen_self.a_imu; 
    // world_a(2) += -9.80665;

    self.F.setIdentity();
    self.F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity(3, 3)*dt;
    // std::cout<< dt << "\n";
    // u.segment(0, 3) = 1/2*dt*dt*world_a;
    u.segment(0, 3) = u.segment(3, 3)* dt; 
    u.segment(3, 3) = world_a*dt;
    ROS_INFO("imu acceleration: %f, %f, %f", world_a(0), world_a(1), world_a(2));
    self.X_hat = self.F*self.X + u;

    self.P_hat = self.F*self.P*self.F.transpose() + Q;
}

void Self_pose_EIF::computeCorrPairs()
{
    
    self.z = measurement;

    self.s.setZero();
    self.y.setZero();

    if(self.z != self.pre_z)
    {
        self.h = self.X_hat.segment(0, 3);
        self.H.block(0, 0, 2, 2).setIdentity();

        self.s = self.H.transpose()*R.inverse()*self.H;
        self.y = self.H.transpose()*R.inverse()*(self.z - self.h + self.H*self.X_hat);
    }

    self.P = (self.P_hat.inverse() + self.s).inverse();
    self.X = self.P*(self.P_hat.inverse()*self.X_hat + self.y);
    self.pre_z = self.z;
}

void Self_pose_EIF::computeCorrPairs(Eigen::Vector2d pixel_z)
{
    double fx, fy, X, Y, Z, cx, cy;
    Eigen::Matrix3d R_B2C, R_w2c;
    Eigen::Vector3d temp;
    Eigen::Matrix2d R_vml;
    self.pre_z.resize(2);
    fx = 565.6008952774197;
    fy = 565.6008952774197;
    cx = 320.5;
    cy = 240.5;
    self.z = pixel_z;
    self.h = pixel_z;
    measurement(2) = 0;

    self.s.setZero();
    self.y.setZero();
    R_B2C << 1, 0, 0,
            0, -1, 0,
            0, 0, -1; 
    R_w2c = R_B2C*Mav_eigen_self.R_w2b; ///////////////// rotation problem
    temp = measurement - self.X_hat.segment(0, 3);
    temp(2) = -40;
    Eigen::Vector3d r_qc_c = R_w2c*(temp); 



    R_vml << 1e-2, 0,
            0, 1e-2;
    self.H.resize(2,6);
 
    if(self.z != self.pre_z)
    {    
        X = r_qc_c(0)/r_qc_c(2);
        Y = r_qc_c(1)/r_qc_c(2);
        Z = r_qc_c(2);
        self.h(0) = fx*X + cx;
		self.h(1) = fy*Y + cy;
        // self.H.block(0, 0, 3, 3).setIdentity();
		self.H(0, 0) = (fx/Z)*(R_w2c(0, 0) - R_w2c(2, 0)*X);
		self.H(0, 1) = (fx/Z)*(R_w2c(0, 1) - R_w2c(2, 1)*X);
		self.H(0, 2) = (fx/Z)*(R_w2c(0, 2) - R_w2c(2, 2)*X);
		self.H(1, 0) = (fy/Z)*(R_w2c(1, 0) - R_w2c(2, 0)*Y);
		self.H(1, 1) = (fy/Z)*(R_w2c(1, 1) - R_w2c(2, 1)*Y);
		self.H(1, 2) = (fy/Z)*(R_w2c(1, 2) - R_w2c(2, 2)*Y);

        self.H = - self.H;
        self.s = self.H.transpose()*R_vml.inverse()*self.H;
        std::cout << self.H<<"test\n";
        self.y = self.H.transpose()*R_vml.inverse()*(self.z - self.h + self.H*self.X_hat);
    }

    self.P = (self.P_hat.inverse() + self.s).inverse();
    self.X = self.P*(self.P_hat.inverse()*self.X_hat + self.y);
    self.pre_z = self.z;
}
EIF_data Self_pose_EIF::getEIFData(){return self;}
void Self_pose_EIF::setFusionPairs(Eigen::MatrixXd fusedP, Eigen::VectorXd fusedX)
{
    self.P = fusedP;
    self.X = fusedX;
}

void Self_pose_EIF::setCurrState(MAV_eigen MAV)
{
    self.X.segment(0, 3) = MAV.r;
    self.X.segment(3, 3) = MAV.v;

    std::cout << "curr_state:\n" << self.X << std::endl;
}
