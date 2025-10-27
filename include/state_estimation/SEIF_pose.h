#ifndef SEIF_POSE_H
#define SEIF_POSE_H

#include "SEIF.h"

class Self_pose_EIF : public Self_EIF
{
private:
    Eigen::VectorXd u;
    Eigen::Vector3d measurement;
    Eigen::Vector3d vo_z;
    Eigen::Matrix2d R_vml;
    Eigen::Matrix3d R_vo;
    Eigen::MatrixXd R_alt;
    Eigen::Vector3d vo_offset;
    double scale;
    double alt_init;
    bool scale_init;
public:
    Self_pose_EIF();
    ~Self_pose_EIF();
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void computeCorrPairs(Eigen::Vector2d pixel_z);
    void setMeasurement(Eigen::Vector3d , Eigen::VectorXd, Eigen::Vector2d);
    void setMapmeasurement(Eigen::Vector2d);
    void setFusionPairs(Eigen::MatrixXd fusedP, Eigen::VectorXd fusedX);
    void setCurrState(MAV_eigen MAV);
    EIF_data getEIFData();
    Eigen::Vector3d getVO();
};


#endif