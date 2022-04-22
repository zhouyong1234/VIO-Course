#include "camera.h"
#include <iostream>


Camera::Camera(const Sophus::SE3d& pose, int id, double focus, double k1, double k2, bool fixed) :pose_(pose), id_(id), focus_(focus), k1_(k1), k2_(k2),fixed_(fixed), state_index_(-1) {}


double Camera::getFocus()
{
    return focus_;
}

double Camera::getK1()
{
    return k1_;
}

double Camera::getK2()
{
    return k2_;
}

const Sophus::SE3d& Camera::getPose()
{
    return pose_;
}

void Camera::setPose(const Sophus::SE3d& pose)
{
    pose_ = pose;
}

int Camera::getId()
{
    return id_;
}

void Camera::setId(int id)
{
    id_ = id;
}

void Camera::setFixed()
{
    fixed_ = true;
}

bool Camera::isFixed()
{
    return fixed_;
}

void Camera::addDeltaPose(const Eigen::Matrix<double, 6, 1>& delta_pose)
{
    Sophus::SE3d delta_SE3d = Sophus::SE3d::exp(delta_pose);
    pose_ = delta_SE3d  * pose_;
}

void Camera::addDeltaPose(const Sophus::SE3d& delta_pose)
{
    pose_ = delta_pose * pose_;
}