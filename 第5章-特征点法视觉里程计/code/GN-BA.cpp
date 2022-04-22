//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <sophus/se3.hpp>

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "/home/touchair/下载/视觉SLAM课程/L5/code/p3d.txt";
string p2d_file = "/home/touchair/下载/视觉SLAM课程/L5/code/p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream p2_in;
    p2_in.open(p2d_file);
    string s;
    while (getline(p2_in, s) && !s.empty())
    {
        istringstream p2d_data(s);
        Vector2d p2d_temp;
        p2d_data >> p2d_temp[0] >> p2d_temp[1];
        p2d.push_back(p2d_temp);
    }
    
    p2_in.close();


    ifstream p3_in;
    p3_in.open(p3d_file);
    while(getline(p3_in, s) && !s.empty())
    {
        istringstream p3d_data(s);
        Vector3d p3d_temp;
        p3d_data >> p3d_temp[0] >> p3d_temp[1] >> p3d_temp[2];
        p3d.push_back(p3d_temp);
    }

    p3_in.close();


    // cout << "p2d.size: " << p2d.size() << " p3d.size: " << p3d.size() << endl;
    

    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 

            Vector3d P = T_esti * p3d[i];

            Vector2d e(p2d[i].x() - (K*P/P[2]).x(), p2d[i].y() - (K*P/P[2]).y());

            cost += 0.5 * e.transpose() * e;
            
            // END YOUR CODE HERE

            // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE


            double X = P[0];
            double Y = P[1];
            double Z = P[2];

            J(0, 0) = -fx/Z;
            J(0, 2) = fx*X/(Z*Z);
            J(0, 3) = fx*X*Y/(Z*Z);
            J(0, 4) = -fx - fx*(X*X)/(Z*Z);
            J(0, 5) =  fx*Y/Z;
            J(1, 1) = -fy/Z;
            J(1, 2) = fy*Y/(Z*Z);
            J(1, 3) = fy + fy*(Y*Y)/(Z*Z);
            J(1, 4) = -fy*X*Y/(Z*Z);
            J(1, 5) = -fy*X/Z;

        
	        // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 

        dx = H.ldlt().solve(b);

        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 

        T_esti = Sophus::SE3d::exp(dx) * T_esti;

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
