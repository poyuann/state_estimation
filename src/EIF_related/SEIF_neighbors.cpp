#include "SEIF_neighbors.h"

Self_rel_EIF::Self_rel_EIF()
{
	self_measurement_size = 3;
    neighbor_num_curr = 0;
	EIF_measurement_init(self_state_size, self_measurement_size, &self);
    
    //////////////////////// Covariance Tuning ////////////////////////

    R(0, 0) = 4e-2;
    R(1, 1) = 4e-2;
    R(2, 2) = 3e-1;
}
Self_rel_EIF::~Self_rel_EIF(){}

void Self_rel_EIF::setmeasurements(Eigen::Vector4d left_CMs, Eigen::Vector4d right_CMs)
{
    left_camerameasurements = left_CMs;
    right_camerameasurements = right_CMs;
}
void Self_rel_EIF::setCamera(Camera CAM1 , Camera CAM2)
{
    cam1 = CAM1;
    cam2 = CAM2;
}
void Self_rel_EIF::setNeighborData(std::vector<EIF_data> robots)
{ 
    neighbor_num_curr = 0;
    neighbor_num_curr = robots.size();
    neighbors_pred = robots;
}

void Self_rel_EIF::setEIFpredData(EIF_data pred)
{
    self = pred;
}

EIF_data Self_rel_EIF::computeCorrPair(Eigen::Vector4d CM, EIF_data& neighbor, Camera cam)
{

    self.z = CM.segment(0, 3);

    self.s.setZero();
    self.y.setZero();

    double X,Y,Z;

    if(checkPreMeasurement(CM))
    {
        Eigen::MatrixXd R_hat;
        Eigen::MatrixXd R_hat_passive;
        Eigen::Matrix3d R_w2c;
        Eigen::MatrixXd neighbor_s;
        Eigen::MatrixXd neighbor_y;

		R_w2c = cam.R_B2C()*Mav_eigen_self.R_w2b; ///////////////// rotation problem
		Eigen::Vector3d r_qc_c = R_w2c*(neighbor.X_hat.segment(0, 3) - self.X_hat.segment(0, 3)); 

		X = r_qc_c(0)/r_qc_c(2);
		Y = r_qc_c(1)/r_qc_c(2);
		Z = r_qc_c(2);

        self.h(0) = cam.fx()*X + cam.cx();
		self.h(1) = cam.fy()*Y + cam.cy();
		self.h(2) = Z;

        neighbor.z = self.z;
        neighbor.h = self.h;
        // std:: cout << "z :"<< self.z.transpose() <<"\n" ;
        // std:: cout << "h :"<< self.h.transpose() <<"\n" ;

        ////////////////////////////////////////////////// derivative w.r.t neighbor //////////////////////////////////////////////////
        neighbor.H.setZero(self_measurement_size, self_state_size);

		neighbor.H(0, 0) = (cam.fx()/Z)*(R_w2c(0, 0) - R_w2c(2, 0)*X);
		neighbor.H(0, 1) = (cam.fx()/Z)*(R_w2c(0, 1) - R_w2c(2, 1)*X);
		neighbor.H(0, 2) = (cam.fx()/Z)*(R_w2c(0, 2) - R_w2c(2, 2)*X);
		neighbor.H(1, 0) = (cam.fy()/Z)*(R_w2c(1, 0) - R_w2c(2, 0)*Y);
		neighbor.H(1, 1) = (cam.fy()/Z)*(R_w2c(1, 1) - R_w2c(2, 1)*Y);
		neighbor.H(1, 2) = (cam.fy()/Z)*(R_w2c(1, 2) - R_w2c(2, 2)*Y);
		neighbor.H(2, 0) = R_w2c(2, 0);
		neighbor.H(2, 1) = R_w2c(2, 1);
		neighbor.H(2, 2) = R_w2c(2, 2);
        ////////////////////////////////////////////////// derivative w.r.t self //////////////////////////////////////////////////
        self.H = -neighbor.H;
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        R_hat = R + neighbor.H*neighbor.P_hat*neighbor.H.transpose();

        self.s = self.H.transpose()*R_hat.inverse()*self.H;
        self.y = self.H.transpose()*R_hat.inverse()*(self.z - self.h + self.H*self.X_hat);

        R_hat_passive = R + self.H*self.P_hat.inverse()*self.H.transpose();
        self.passive_s.push_back(neighbor.H.transpose()*R_hat_passive.inverse()*neighbor.H);
        self.passive_y.push_back(neighbor.H.transpose()*R_hat_passive.inverse()*(self.z - self.h + neighbor.H*neighbor.X_hat));

        self.passive_id.push_back(neighbor.ID);
        // std::cout << self.passive_s[0]<< "\n";
        // neighbor_s = neighbor.H.transpose()*R_hat_passive.inverse()*neighbor.H;
        // neighbor_y = neighbor.H.transpose()*R_hat_passive.inverse()*(self.z - self.h + neighbor.H*neighbor.X_hat);
        // set_passive_measure(neighbor_s, neighbor_y);
    }

    setPreMeasurement(CM);
    return self;
}

void Self_rel_EIF::computeCorrPairs()
{
    selfWRTneighbors.clear();
    for(int i=0; i< neighbor_num_curr; i++)
    {
                // std::cout << "ID " <<neighbors_pred[i].ID<<"\n \n";

        if(left_camerameasurements(3) == neighbors_pred[i].ID)
        {
            selfWRTneighbors.push_back(computeCorrPair(left_camerameasurements, neighbors_pred[i], cam1));
        }
        if(right_camerameasurements(3) == neighbors_pred[i].ID)
        {
            selfWRTneighbors.push_back(computeCorrPair(right_camerameasurements, neighbors_pred[i], cam2));
        }
    }
}

EIF_data Self_rel_EIF::getselfEIFData(){return self;}

std::vector<EIF_data> Self_rel_EIF::getEIFData(){ return selfWRTneighbors;}

void Self_rel_EIF::setPreMeasurement(Eigen::Vector4d CM)
{
    if(pre_camerameasurements.size() == 0)
    {
        pre_camerameasurements.push_back(CM);
    }
    else
    {
        bool found = false;
        for(int i=0; i< pre_camerameasurements.size(); i++)
        {
            if(pre_camerameasurements[i](3) == CM(3))
            {
                pre_camerameasurements[i] = CM;
                found = true;
                break;
            }    
        }
        if(!found)
            pre_camerameasurements.push_back(CM);
    }
}

bool Self_rel_EIF::checkPreMeasurement(Eigen::Vector4d CM)
{
    if(pre_camerameasurements.size() > 0)
    {
        for(int i=0; i<pre_camerameasurements.size(); i++)
        {
            if(pre_camerameasurements[i](3) == CM(3))
            {
                if(pre_camerameasurements[i].segment(0, 3) == CM.segment(0, 3) )
                    return false;
                else
                    return true;
            }
        }
    }
    return true;
}

