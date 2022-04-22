#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include "mappoint.h"
#include "camera.h"

class CostFunction
{
private:
    Eigen::Vector2d ob_z_;
    Eigen::Matrix2d info_matrix_;
    double huber_b_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CostFunction(MapPoint* map_point, Camera* camera, const Eigen::Vector2d& ob_z);

    void setHuberParameter(double b = 1.0);
    void setCovariance(const Eigen::Matrix2d& cov);

    void computeInterVars(Eigen::Vector2d& e, Eigen::Matrix2d& weighted_info, double& weighted_e2);

    void computeJT(Eigen::Matrix<double, 2, 6>& JT);
    void computeJX(Eigen::Matrix<double, 2, 3>& JX);

    MapPoint* map_point_;
    Camera* camera_;
};



#endif