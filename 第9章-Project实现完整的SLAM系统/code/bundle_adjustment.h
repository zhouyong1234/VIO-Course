#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

#include "mappoint.h"
#include "camera.h"
#include "cost_function.h"

#include <map>
#include <set>
#include <vector>

class BundleAdjustment
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    void optimizationInit();
    void computeStateIndexes();
    void computeHAndbAndError();
    void solveNormalEquation();
    void inverseM(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_inv);
    void updateStates();
    void recoverStates();

    std::map<int, MapPoint*> mappoints_;
    std::map<int, Camera*> cameras_;
    std::set<CostFunction*> cost_functions_;

    Eigen::MatrixXd J_;
    Eigen::MatrixXd JTinfo_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd r_;
    Eigen::MatrixXd b_;
    Eigen::MatrixXd info_matrix_;
    Eigen::MatrixXd Delta_X_;
    Eigen::MatrixXd I_;
    double lambda_;

    int n_cam_state_;
    int n_mpt_state_;

    int max_iters_;
    double min_delta_;
    double min_error_;
    double sum_error2_;
    double last_sum_error2_;

    bool verbose_;


public:
    BundleAdjustment();
    ~BundleAdjustment();

    void addMapPoint(MapPoint* mpt);
    void addCamera(Camera* cam);
    void addCostFunction(CostFunction* cost_func);
    MapPoint* getMapPoint(int id);
    Camera* getCamera(int id);
    void setConvergenceCondition(int max_iters, double min_delta, double min_error);
    void setVerbose(bool flag);
    void optimize();
};




#endif