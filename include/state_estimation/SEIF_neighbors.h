#ifndef SEIF_NIEGHBORS_H
#define SEIF_NIEGHBORS_H

#include "SEIF.h"
#include "Camera.h"
class Self_rel_EIF : public Self_EIF
{
private:
    MAV_eigen mav_self_data;
    std::vector<EIF_data> neighbors_pred;
    std::vector<EIF_data> selfWRTneighbors;
    std::vector<Eigen::Vector4d> camerameasurements;
    std::vector<Eigen::Vector4d> pre_camerameasurements;
    Eigen::Vector4d left_camerameasurements;
    Eigen::Vector4d right_camerameasurements;
    int neighbor_num_curr;
    Camera cam1;
    Camera cam2;
    EIF_data passive;
public:
    Self_rel_EIF();
    ~Self_rel_EIF();
    void setNeighborData(std::vector<EIF_data> robots);
    void setmeasurements(Eigen::Vector4d left_CMs, Eigen::Vector4d right_CMs);
    void setEIFpredData(EIF_data pred);
    void setCamera(Camera , Camera);
    EIF_data getselfEIFData();
    EIF_data computeCorrPair(Eigen::Vector4d CM, EIF_data& neighbor, Camera cam);
    void computeCorrPairs();
    std::vector<EIF_data> getEIFData();
    void setPreMeasurement(Eigen::Vector4d CM);
    bool checkPreMeasurement(Eigen::Vector4d CM);
};


#endif