#include "SEIF_lidar_neighbors.h"

Self_lidar_EIF::Self_lidar_EIF()
{
	self_measurement_size = 3;
    neighbor_num_curr = 0;
	EIF_measurement_init(self_state_size, self_measurement_size, &self);
    
    //////////////////////// Covariance Tuning ////////////////////////

    // R(0, 0) = 3e-4;
    // R(1, 1) = 6e-5;
    // R(2, 2) = 6e-5;

    R(0, 0) = 3e-4;
    R(1, 1) = 2e-2;
    R(2, 2) = 2e-2;
    
}
Self_lidar_EIF::~Self_lidar_EIF(){}

void Self_lidar_EIF::setLidarMeasurements(std::vector<Eigen::Vector4d> LMs)
{
    lidarMeasurements = LMs;

}

void Self_lidar_EIF::setNeighborData(std::vector<EIF_data> robots)
{ 
    neighbor_num_curr = 0;
    neighbor_num_curr = robots.size();

    std::cout << "Current neighbor number: " << neighbor_num_curr << "\n";
    neighbors_pred = robots;
}

void Self_lidar_EIF::setEIFpredData(EIF_data pred)
{
    self = pred;
}

EIF_data Self_lidar_EIF::computeCorrPair(Eigen::Vector4d LM, EIF_data& neighbor)
{
    self.z = LM.segment(0, 3);

    self.s.setZero();
    self.y.setZero();
    std::cout << "Lidar measurement: " << self.z.transpose() << "\n";
    if(checkPreMeasurement(LM) && isnormal(LM(0)))
    {
        std::cout << "Lidar measurement received for neighbor ID " << LM(3) << "\n";
        Eigen::MatrixXd R_hat;
        Eigen::Matrix3d R_W2B = Mav_eigen_self.R_w2b;
        Eigen::Vector3d r_B_hat = R_W2B*(neighbor.X_hat.segment(0, 3) - self.X_hat.segment(0, 3));
        Eigen::MatrixXd R_hat_passive;
        
        double D = sqrt(pow(r_B_hat(0), 2) + pow(r_B_hat(1), 2) + pow(r_B_hat(2), 2));
        
        self.h.resize(3);  // Ensure size is at least 3
        self.h(0) = D;
        std::cout << D << "\n";
        self.h(1) = std::acos(r_B_hat(2)/D);
        std::cout << r_B_hat(2)/D << "\n";
        std::cout << r_B_hat(1) << "," << r_B_hat(0) << "\n";        
        self.h(2) = std::atan2(r_B_hat(1), r_B_hat(0));
        std::cout << "exit code -6\n";
        ////////////////////////////////////////////////// derivative w.r.t neighbor //////////////////////////////////////////////////
        neighbor.H.setZero(self_measurement_size, self_state_size);

        neighbor.H(0, 0) = (R_W2B(0, 0)*r_B_hat(0) + R_W2B(1, 0)*r_B_hat(1) + R_W2B(2, 0)*r_B_hat(2)) / D;
        neighbor.H(0, 1) = (R_W2B(0, 1)*r_B_hat(0) + R_W2B(1, 1)*r_B_hat(1) + R_W2B(2, 1)*r_B_hat(2)) / D;
        neighbor.H(0, 2) = (R_W2B(0, 2)*r_B_hat(0) + R_W2B(1, 2)*r_B_hat(1) + R_W2B(2, 2)*r_B_hat(2)) / D;
        
        neighbor.H(1, 0) = (R_W2B(0, 0)*r_B_hat(0)*r_B_hat(2)
                                + R_W2B(1, 0)*r_B_hat(1)*r_B_hat(2)
                                - R_W2B(2, 0)*(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)))
                                /(D*D * sqrt(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)));
        neighbor.H(1, 1) = (R_W2B(0, 1)*r_B_hat(0)*r_B_hat(2)
                                + R_W2B(1, 1)*r_B_hat(1)*r_B_hat(2)
                                - R_W2B(2, 1)*(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)))
                                /(D*D * sqrt(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)));
        neighbor.H(1, 2) = (R_W2B(0, 2)*r_B_hat(0)*r_B_hat(2)
                                + R_W2B(1, 2)*r_B_hat(1)*r_B_hat(2)
                                - R_W2B(2, 2)*(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)))
                                /(D*D * sqrt(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)));

        neighbor.H(2, 0) = (-R_W2B(0, 0)*r_B_hat(1) + R_W2B(1, 0)*r_B_hat(0)) / (r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1));
        neighbor.H(2, 1) = (-R_W2B(0, 1)*r_B_hat(1) + R_W2B(1, 1)*r_B_hat(0)) / (r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1));
        neighbor.H(2, 2) = (-R_W2B(0, 2)*r_B_hat(1) + R_W2B(1, 2)*r_B_hat(0)) / (r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1));

        ////////////////////////////////////////////////// derivative w.r.t self //////////////////////////////////////////////////
        self.H = -neighbor.H;
        std::cout << "Jacobian computed for neighbor ID: " << LM(3) << "\n";
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        R_hat = R + neighbor.H*neighbor.P_hat*neighbor.H.transpose();
        std::cout << "R_hat:\n" << R_hat << "\n";
        self.s = self.H.transpose()*R_hat.inverse()*self.H;
        self.y = self.H.transpose()*R_hat.inverse()*(self.z - self.h + self.H*self.X_hat);
        std::cout << "EIF Correction computing for neighbor ID: " << LM(3) << "\n";
        // R_hat_passive = R + self.H*self.P_hat.inverse()*self.H.transpose();
        // self.passive_s.push_back(neighbor.H.transpose()*R_hat_passive.inverse()*neighbor.H);
        // self.passive_y.push_back(neighbor.H.transpose()*R_hat_passive.inverse()*(self.z - self.h + neighbor.H*neighbor.X_hat));

        // self.passive_id.push_back(neighbor.ID);
    }
    setPreMeasurement(LM);
    std::cout << "EIF Correction done for neighbor ID: " << LM(3) << "\n\n";
    return self;
}

void Self_lidar_EIF::computeCorrPairs()
{
    selfWRTneighbors.clear();
    for(int i=0; i< neighbor_num_curr; i++)
    {
        for(int j=0; j<lidarMeasurements.size(); j++)
            if(lidarMeasurements[j](3) == neighbors_pred[i].ID)
            {
                selfWRTneighbors.push_back(computeCorrPair(lidarMeasurements[j], neighbors_pred[i]));
                break;
            }
    }
}

EIF_data Self_lidar_EIF::getselfEIFData(){return self;}

std::vector<EIF_data> Self_lidar_EIF::getEIFData(){ return selfWRTneighbors;}

void Self_lidar_EIF::setPreMeasurement(Eigen::Vector4d LM)
{
    if(pre_lidarMeasurements.size() == 0)
    {
        pre_lidarMeasurements.push_back(LM);
    }
    else
    {
        bool found = false;
        for(int i=0; i< pre_lidarMeasurements.size(); i++)
        {
            if(pre_lidarMeasurements[i](3) == LM(3))
            {
                pre_lidarMeasurements[i] = LM;
                found = true;
                break;
            }    
        }
        if(!found)
            pre_lidarMeasurements.push_back(LM);
    }
}

bool Self_lidar_EIF::checkPreMeasurement(Eigen::Vector4d LM)
{
    if(pre_lidarMeasurements.size() > 0)
    {
        for(int i=0; i<pre_lidarMeasurements.size(); i++)
        {
            if(pre_lidarMeasurements[i](3) == LM(3))
            {
                if(pre_lidarMeasurements[i].segment(0, 3) == LM.segment(0, 3))
                    return false;
                else
                    return true;
            }
        }
    }
    return true;
}