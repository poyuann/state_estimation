#include "SEIF_pose.h"

Self_pose_EIF::Self_pose_EIF()
{
	self_measurement_size = 3;
    EIF_measurement_init(self_state_size, self_measurement_size, &self);
    u.setZero(self_state_size);
    measurement.setZero();
    self.z.setZero(2);
    self.z_vo.setZero(3);
    self.z_alt.setZero(1);
    self.pre_z.setZero(2);
    self.pre_z_vo.setZero(3);
    self.pre_z_alt.setZero(1);
    self.H_alt.setZero(1, self_state_size);
    self.H_vo.setZero(3, self_state_size);
    self.h_alt.setZero(1);
    self.h_vo.setZero(3);
    alt_init = 0.0;
    //////////////////////// Covariance Tuning ////////////////////////
    vo_offset.setZero(3);
    vo_offset = self.X.segment(0,3);
    scale_init = false;
    R_vml = 1e2*Eigen::Matrix2d::Identity();
    R_vo = 1*Eigen::Matrix3d::Identity();
    R_alt = 1*Eigen::MatrixXd::Identity(1,1);
    R = 4e2*Eigen::MatrixXd::Identity(self_measurement_size, self_measurement_size);
    R(2,2) = 1e2;
}
Self_pose_EIF::~Self_pose_EIF(){}

void Self_pose_EIF::setMeasurement(Eigen::Vector3d z_vo, Eigen::VectorXd z_alt, Eigen::Vector2d z_vml)
{
    self.z_vo = z_vo;
    self.z_alt(0) = z_alt(0);
    self.z = z_vml;
}

void Self_pose_EIF::setMapmeasurement(Eigen::Vector2d z)
{
    self.z = z;
}

void Self_pose_EIF::computePredPairs(double delta_t)
{
    self.pre_X = self.X.segment(0, 3);
    double dt = static_cast<double>(delta_t);
    // double dt = 0.001;
    Eigen::Vector3d world_a = Mav_eigen_self.R_w2b.inverse()*Mav_eigen_self.a_imu; 
    world_a(2) += -9.80665;

    self.F.setIdentity();
    self.F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity(3, 3)*dt;
    // u.segment(0, 3) = 1/2*dt*dt*world_a;
    u.segment(0, 3) = u.segment(3, 3)* dt; 
    u.segment(3, 3) = world_a*dt;
    self.X_hat = self.F*self.X + u;

    self.P_hat = self.F*self.P*self.F.transpose() + Q;
}

void Self_pose_EIF::computeCorrPairs()
{    
    self.s.setZero();
    self.y.setZero();
    self.s_alt.setZero(self_state_size, self_state_size);
    self.y_alt.setZero(self_state_size);
    self.s_vo.setZero(self_state_size, self_state_size);
    self.y_vo.setZero(self_state_size);

    if(self.z != self.pre_z)
    {
        self.H.setZero(2, self_state_size);
        self.h = self.X_hat.segment(0, 2);
        self.H.block(0, 0, 2, 2).setIdentity();
        self.s = self.H.transpose()*R_vml.inverse()*self.H;
        self.y = self.H.transpose()*R_vml.inverse()*(self.z - self.h + self.H*self.X_hat);
        self.pre_z = self.z;
    }
    if(self.z_vo != self.pre_z_vo)
    {
        // vo_z = self.z_vo - self.pre_z_vo;

        vo_z = self.z_vo;
        if (self.z_alt(0) < 70)
            scale = (self.z_alt(0) - alt_init)/ (vo_z(2) + 1e-6);
        // vo_z = (vo_z- self.pre_z_vo) * scale;
        // self.h_vo = self.X_hat.segment(0, 3) - self.pre_X;
        // std::cout << "vo_z"<<vo_z<<"\n";
        // std::cout << "h_vo"<<vo_z - self.h_vo<<"\n";
        vo_z = vo_z * scale + vo_offset;
        if(!scale_init)
        {
            vo_offset = self.X_hat.segment(0,3) - vo_z;
            scale_init = true;
            vo_z = vo_z + vo_offset;
        }
        self.h_vo = self.X_hat.segment(0, 3);
        self.H_vo.block(0, 0, 3, 3).setIdentity();
        self.s_vo = self.H_vo.transpose()*R_vo.inverse()*self.H_vo;
        self.y_vo = self.H_vo.transpose()*R_vo.inverse()*(vo_z - self.h_vo + self.H_vo*self.X_hat);
        self.pre_z_vo = self.z_vo;

    }
    if(self.pre_z_alt != self.z_alt)
    {
        self.h_alt(0) = self.X_hat(2);
        self.H_alt(0, 2) = 1;

        self.s_alt = self.H_alt.transpose()*R_alt.inverse()*self.H_alt;
        self.y_alt = self.H_alt.transpose()*R_alt.inverse()*(self.z_alt - self.h_alt + self.H_alt*self.X_hat);
        self.pre_z_alt = self.z_alt;
    }
    self.P = (self.P_hat.inverse() + self.s + self.s_alt +self.s_vo).inverse();
    self.X = self.P*(self.P_hat.inverse()*self.X_hat + (self.y + self.y_alt + self.y_vo));
    self.P_hat = self.P;
    self.X_hat = self.X;
    // std::cout << "Updated State:\n" << self.X << std::endl;  
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
Eigen::Vector3d Self_pose_EIF::getVO()
{
    return self.z_vo * scale;
}