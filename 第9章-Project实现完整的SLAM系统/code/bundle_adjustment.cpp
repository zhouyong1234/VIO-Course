#include "bundle_adjustment.h"

#include <Eigen/Core>
#include <iostream>
#include <iomanip>
#include "run_timer.h"

BundleAdjustment::BundleAdjustment() : max_iters_(20), min_delta_(1e-10), min_error_(1e-10), last_sum_error2_(std::numeric_limits<double>::max()),  verbose_(false) {}

BundleAdjustment::~BundleAdjustment()
{
    std::map<int, Camera*>::iterator it_cam;
    for(it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam++)
    {
        Camera* cam = it_cam->second;
        delete cam;
    }

    std::map<int, MapPoint*>::iterator it_mpt;
    for(it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++)
    {
        MapPoint* mpt = it_mpt->second;
        delete mpt;
    }

    std::set<CostFunction*>::iterator it_cost;
    for(it_cost = cost_functions_.begin(); it_cost != cost_functions_.end(); it_cost++)
    {
        CostFunction* cost_func = *it_cost;
        delete cost_func;
    }
}

void BundleAdjustment::addMapPoint(MapPoint* mpt)
{
    mappoints_[mpt->getId()] = mpt;
}

void BundleAdjustment::addCamera(Camera* cam)
{
    cameras_[cam->getId()] = cam;
}

void BundleAdjustment::addCostFunction(CostFunction* cost_func)
{
    cost_functions_.insert(cost_func);
}

Camera* BundleAdjustment::getCamera(int id)
{
    std::map<int, Camera*>::iterator iter = cameras_.find(id);
    if(iter != cameras_.end())
    {
        return iter->second;
    }
    return nullptr;
}

MapPoint* BundleAdjustment::getMapPoint(int id)
{
    std::map<int, MapPoint*>::iterator iter = mappoints_.find(id);
    if(iter != mappoints_.end())
    {
        return iter->second;
    }
    return nullptr;
}

void BundleAdjustment::setConvergenceCondition(int max_iters, double min_delta, double min_error)
{
    max_iters_ = max_iters;
    min_delta_ = min_delta;
    min_error_ = min_error;
}

void BundleAdjustment::setVerbose(bool verbose_flag)
{
    verbose_ = verbose_flag;
}

void BundleAdjustment::optimize()
{
    optimizationInit();

    if(true)
    {
        int niter = 0;
        double upsilon = 2.0;
        const double tau = 1e-8;

        computeHAndbAndError();
        last_sum_error2_ = sum_error2_;

        bool found = ((-b_).lpNorm<Eigen::Infinity>() < min_error_);

        // std::cout << b_.transpose() << std::endl;
        // std::cout << found << std::endl;

        std::vector<double> aa;
        aa.reserve(H_.rows());
        for(int i = 0; i < H_.rows(); i++)
        {
            aa.push_back(H_(i, i));
        }

        auto max_aa = std::max_element(aa.begin(), aa.end());
        double mu = tau * (*max_aa);

        double total_time = 0.0;
        while (!found && niter < max_iters_)
        {
            Runtimer t;
            t.start();

            niter++;

            H_ += mu * I_;
            solveNormalEquation();

            double delta = Delta_X_.norm();

            if(delta < min_delta_)
            {
                break;
            }

            updateStates();

            computeHAndbAndError();

            double varrho = (last_sum_error2_ - sum_error2_) / (Delta_X_.transpose()*(mu*Delta_X_+b_))(0,0);

            // 误差一直在减小
            if(varrho > 0)
            {
                last_sum_error2_ = sum_error2_;
                found = ((-b_).lpNorm<Eigen::Infinity>() < min_error_);
                mu = mu * std::max<double>(0.3333, 1.0-std::pow(2.0*varrho-1.0, 3));
                upsilon = 2.0;
            }
            else
            {
                recoverStates();
                computeHAndbAndError();
                mu = mu * upsilon;
                upsilon *= 2.0;
            }

            t.stop();
            total_time += t.duration();

            if(verbose_)
            {
                std::cout << std::fixed << "Iter: " << std::left <<std::setw(4) << niter 
                << " Cost: "<< std::left <<std::setw(20)  << std::setprecision(10) << sum_error2_ 
                << " Step: " << std::left <<std::setw(14) << std::setprecision(10) << delta 
                << " Time " << std::left <<std::setw(10) << std::setprecision(3) << t.duration() 
                << " Total_time " << std::left <<std::setw(10) << std::setprecision(3) << total_time << std::endl;
            }

        }
    }
}

void BundleAdjustment::optimizationInit()
{
    computeStateIndexes();
    
    int state_size = n_cam_state_ * 6 + n_mpt_state_ * 3;
    int obs_size = cost_functions_.size() * 2;
    // std::cout << state_size << " " << obs_size << std::endl;
    J_.resize(obs_size, state_size);
    JTinfo_.resize(state_size, obs_size);
    H_.resize(state_size, state_size);
    r_.resize(obs_size, 1);
    b_.resize(state_size, 1);
    // info_matrix_.resize(obs_size, obs_size);
    Delta_X_.resize(state_size, 1);
    I_.resize(state_size, state_size);
    for(int i = 0; i < state_size; i++)
    {
        I_(i,i) = 1.0;
    }
}

void BundleAdjustment::computeStateIndexes()
{
    int INDEX = 0;
    n_cam_state_ = 0;
    std::map<int, Camera*>::iterator it_cam;
    for(it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam++)
    {
        Camera* cam = it_cam->second;
        if(cam->isFixed() == false)
        {
            cam->state_index_ = INDEX;
            INDEX++;
            n_cam_state_++;
        }
        else
        {
            cam->state_index_ = -1;
        }
    }

    INDEX = 0;
    n_mpt_state_ = 0;
    std::map<int, MapPoint*>::iterator it_mpt;
    for(it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++)
    {
        MapPoint* mpt = it_mpt->second;
        if(mpt->isFixed() == false)
        {
            mpt->state_index_ = INDEX;
            INDEX++;
            n_mpt_state_++;
        }
        else
        {
            mpt->state_index_ = -1;
        }
    }
}


void BundleAdjustment::computeHAndbAndError()
{
    int cnt = 0;
    J_.setZero();
    H_.setZero();
    r_.setZero();
    info_matrix_.setIdentity();
    JTinfo_.setZero();

    sum_error2_ = 0;
    std::set<CostFunction*>::iterator iter;
    for(iter = cost_functions_.begin(); iter != cost_functions_.end(); iter++)
    {
        CostFunction* cost_func = *iter;

        Camera* cam = cost_func->camera_;
        if(cam->isFixed() == false)
        {
            Eigen::Matrix<double, 2, 6> JT;
            cost_func->computeJT(JT);
            J_.block<2,6>(2*cnt, cam->state_index_*6) = JT;
            // std::cout << JT << std::endl;
        }

        

        MapPoint* mpt = cost_func->map_point_;
        if(mpt->isFixed() == false)
        {
            Eigen::Matrix<double, 2, 3> JX;
            cost_func->computeJX(JX);
            J_.block<2,3>(2*cnt, n_cam_state_*6 + mpt->state_index_*3) = JX;
            // std::cout << JX << std::endl;
        }

        Eigen::Vector2d e;
        Eigen::Matrix2d weighted_info;
        double weighted_e2;
        cost_func->computeInterVars(e, weighted_info, weighted_e2);
        r_.block<2,1>(2*cnt, 0) = e;
        // std::cout << e.transpose() << std::endl;
        // info_matrix_.block<2,2>(2*cnt, 2*cnt) = weighted_info;
        sum_error2_ += weighted_e2;
        cnt++;
    }

    JTinfo_ = J_.transpose();
    H_ = JTinfo_ * J_;
    b_ = -JTinfo_ * r_;
}


void BundleAdjustment::solveNormalEquation()
{
    if(n_cam_state_ >= 1 && n_mpt_state_ >= 1)
    {
        // Solve by schur
        // [C  E
        //  ET M]
        const Eigen::MatrixXd& C = H_.block ( 0, 0, n_cam_state_*6, n_cam_state_*6 );
        const Eigen::MatrixXd& M = H_.block ( n_cam_state_*6, n_cam_state_*6, n_mpt_state_*3, n_mpt_state_*3 );
		const Eigen::MatrixXd& ET = H_.block ( n_cam_state_*6, 0, n_mpt_state_*3, n_cam_state_*6);
		const Eigen::MatrixXd& E = H_.block ( 0, n_cam_state_*6, n_cam_state_*6, n_mpt_state_*3);

        const Eigen::MatrixXd& bc = b_.block(0,0, n_cam_state_*6, 1);
        const Eigen::MatrixXd& bm = b_.block(n_cam_state_*6, 0, n_mpt_state_*3, 1);

        Eigen::MatrixXd M_inv;
        inverseM(M, M_inv);
        // Solve Delta X_c
        Delta_X_.block(0,0, n_cam_state_*6, 1) = (C-E*M_inv*ET).fullPivHouseholderQr().solve(bc-E*M_inv*bm);
        // Solve Delta X_m
        Delta_X_.block(n_cam_state_*6, 0, n_mpt_state_*3, 1) = M_inv*(bm-ET*Delta_X_.block(0,0, n_cam_state_*6, 1));

    }
    else
    {
        Delta_X_ = H_.llt().solve(b_);
    }
}

void BundleAdjustment::inverseM(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_inv)
{
    M_inv = M;
    for(int i = 0; i < n_mpt_state_; i++)
    {
        M_inv.block(3*i, 3*i, 3, 3) = M_inv.block(3*i, 3*i, 3, 3).inverse();
    }
}

void BundleAdjustment::updateStates()
{
    std::map<int, Camera*>::iterator it_cam;
    for(it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam++)
    {
        Camera* cam = it_cam->second;
        if(cam->isFixed() == false)
        {
            int index = cam->state_index_;
            cam->addDeltaPose(Delta_X_.block(index*6, 0, 6, 1));
        }
    }

    std::map<int, MapPoint*>::iterator it_mpt;
    for(it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++)
    {
        MapPoint* mpt = it_mpt->second;
        if(mpt->isFixed() == false)
        {
            int index = mpt->state_index_;
            mpt->addDeltaPosition(Delta_X_.block(n_cam_state_*6+index*3, 0, 3, 1));
        }
    }
}

void BundleAdjustment::recoverStates()
{
    std::map<int, Camera*>::iterator it_cam;
    for(it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam++)
    {
        Camera* cam = it_cam->second;
        if(cam->isFixed() == false)
        {
            int index = cam->state_index_;
            cam->addDeltaPose(-Delta_X_.block(index*6, 0, 6, 1));
        }
    }

    std::map<int, MapPoint*>::iterator it_mpt;
    for(it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++)
    {
        MapPoint* mpt = it_mpt->second;
        if(mpt->isFixed() == false)
        {
            int index = mpt->state_index_;
            mpt->addDeltaPosition(-Delta_X_.block(n_cam_state_*6+index*3, 0, 3, 1));
        }
    }
}