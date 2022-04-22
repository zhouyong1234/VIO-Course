#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include <sophus/se3.hpp>

class Camera
{
private:
    Sophus::SE3d pose_;
    double focus_;
    double k1_, k2_;
    int id_;
    bool fixed_;
public:
    Camera(const Sophus::SE3d& pose, int id, double focus, double k1, double k2, bool fixed = false);
    const Sophus::SE3d& getPose();
    double getFocus();
    double getK1();
    double getK2();
    void setPose(const Sophus::SE3d& pose);
    int getId();
    void setId(int id);
    void setFixed();
    bool isFixed();
    void addDeltaPose(const Eigen::Matrix<double, 6, 1>& delta_pose);
    void addDeltaPose(const Sophus::SE3d& delta_pose);

    int state_index_;
};



#endif
