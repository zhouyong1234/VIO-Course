#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/so3.hpp>

 
using namespace std;


int main()
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1));
    R = rotation_vector.toRotationMatrix();
    Eigen::Quaterniond q(R);

    cout << "R is: " << endl << R << "\n" << endl;

    // cout << q.toRotationMatrix() << "\n" << endl;

    Eigen::Vector3d w(0.01, 0.02, 0.03);
    Sophus::SO3d SO3d_w = Sophus::SO3d::exp(w);
    Eigen::Matrix3d R_w = SO3d_w.matrix();
    Eigen::Quaterniond q_w(1, w[0]/2, w[1]/2, w[2]/2);
    
    cout << "By R: " << endl << R*R_w << "\n" << endl;
    cout << "By q: " << endl << (q*q_w).toRotationMatrix() << endl;


    return 0;
}

