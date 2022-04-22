//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<MatrixXd> svd(E, ComputeThinU | ComputeThinV);

    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    Matrix3d Sigma = U.inverse() * E * V.transpose().inverse();


    vector<double> diag = {Sigma(0,0), Sigma(1,1), Sigma(2,2)};
    sort(diag.begin(), diag.end());

    // for(int i = 0; i < 3; i++)
    // {
    //     cout << diag[i] << endl;
    // }

    // cout << "U = " << endl << U << endl;
    // cout << "V = " << endl << V << endl;

    // cout << "Sigma = " << endl << Sigma << endl;

    double tau = 0.5 * (diag[1] + diag[2]);

    Matrix3d Sigma_tau = Matrix3d::Zero();

    Sigma_tau(0,0) = tau;
    Sigma_tau(1,1) = tau;


    Matrix3d R_z1 = AngleAxisd(M_PI/2, Vector3d(0,0,1)).matrix();
    Matrix3d R_z2 = AngleAxisd(-M_PI/2, Vector3d(0,0,1)).matrix();

    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;


    t_wedge1 = U * R_z1 * Sigma_tau * U.transpose();
    t_wedge2 = U * R_z2 * Sigma_tau * U.transpose();

    // cout << "U = " << endl << U << endl;
    // cout << "V = " << endl << V << endl;

    // cout << "R_z1: " << endl << R_z1 << endl;
    // cout << "R_z2: " << endl << R_z2 << endl;

    // cout << "Sigma_tau: " << endl << Sigma_tau << endl;

    // cout << "t_wedge1: " << endl << t_wedge1 << endl;
    // cout << "t_wedge2: " << endl << t_wedge2 << endl;

    R1 = U * R_z1 * V.transpose();
    R2 = U * R_z2 * V.transpose();

    // END YOUR CODE HERE

    cout << "R1 = " << endl << R1 << endl;
    cout << "R2 = " << endl << R2 << endl;
    cout << "t1 = " << endl << Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "t2 = " << endl << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << endl << tR << endl;

    return 0;
}