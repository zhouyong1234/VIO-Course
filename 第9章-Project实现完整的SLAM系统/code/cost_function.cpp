#include "cost_function.h"

CostFunction::CostFunction(MapPoint* map_point, Camera* camera, const Eigen::Vector2d& ob_z) : map_point_(map_point), camera_(camera), ob_z_(ob_z), huber_b_(1.0), info_matrix_(Eigen::Matrix2d::Identity()) {}


void CostFunction::setHuberParameter(double b)
{
    huber_b_ = b;
}

void CostFunction::setCovariance(const Eigen::Matrix2d& cov)
{
    info_matrix_ = cov.inverse();
}

void CostFunction::computeInterVars(Eigen::Vector2d& e, Eigen::Matrix2d& weighted_info, double& weighted_e2)
{
    const Eigen::Vector3d ptc = camera_->getPose() * map_point_->getPosition();
    Eigen::Vector2d p = -Eigen::Vector2d(ptc[0]/ptc[2],ptc[1]/ptc[2]);
    double r_p = 1.0 + camera_->getK1() * p.squaredNorm() + camera_->getK2() * p.squaredNorm() * p.squaredNorm();
    Eigen::Vector2d u(camera_->getFocus()*r_p*p[0], camera_->getFocus()*r_p*p[1]);
    e = ob_z_ - u;
    
    double et_info_e = e.transpose() * info_matrix_ * e;

    // std::cout << "et_info_e: " << et_info_e << std::endl;
    // std::cout << "e: " << e.transpose() << std::endl;

    double weight = 1.0;
    double sqrt_ete = sqrt(et_info_e);
    if(sqrt_ete > huber_b_)
    {
        weight = 2*huber_b_*sqrt_ete - huber_b_ * huber_b_;
        weight = weight / et_info_e;
    }

    weighted_info = weight * info_matrix_;
    weighted_e2 = weight * et_info_e;

}


void CostFunction::computeJT(Eigen::Matrix<double, 2, 6>& JT)
{
    const Eigen::Vector3d ptc = camera_->getPose() * map_point_->getPosition();
    Eigen::Vector2d p = -Eigen::Vector2d(ptc[0]/ptc[2],ptc[1]/ptc[2]);
    double f = camera_->getFocus();
    double r_p = 1.0 + camera_->getK1() * p.squaredNorm() + camera_->getK2() * p.squaredNorm() * p.squaredNorm();
    const double& x = ptc(0);
    const double& y = ptc(1);
    const double& z = ptc(2);

    JT.setZero();
    const double z2 = z*z;

    JT(0,0) = f*r_p/z;
    JT(0,1) = 0;
    JT(0,2) = -f*r_p*x/z2;
    JT(0,3) = -f*r_p*x*y/z2;
    JT(0,4) = f*r_p+f*r_p*x*x/z2;
    JT(0,5) = -f*r_p*y/z;

    JT(1,0) = 0;
    JT(1,1) = f*r_p/z;
    JT(1,2) = -f*r_p*y/z2;
    JT(1,3) = -f*r_p-f*r_p*y*y/z2;
    JT(1,4) = f*r_p*x*y/z2;
    JT(1,5) = f*r_p*x/z;
}


void CostFunction::computeJX(Eigen::Matrix<double, 2, 3>& JX)
{
    const Eigen::Vector3d ptc = camera_->getPose() * map_point_->getPosition();
    Eigen::Vector2d p = -Eigen::Vector2d(ptc[0]/ptc[2],ptc[1]/ptc[2]);
    double f = camera_->getFocus();
    double r_p = 1.0 + camera_->getK1() * p.squaredNorm() + camera_->getK2() * p.squaredNorm() * p.squaredNorm();
    const double& x = ptc(0);
    const double& y = ptc(1);
    const double& z = ptc(2);

    JX.setZero();

    Eigen::Matrix<double, 2, 3> Jtmp;
    Jtmp.setZero();

    Jtmp(0,0) = f*r_p / z;
    Jtmp(0,1) = 0;
    Jtmp(0,2) = -f*r_p*x/(z*z);
    Jtmp(1,0) = 0;
    Jtmp(1,1) = f*r_p/z;
    Jtmp(1,2) = -f*r_p*y/(z*z);

    JX = Jtmp * camera_->getPose().rotationMatrix();

}
