#include <iostream>

using namespace std;

#include <ctime>
// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 50

/****************************
* 本程序演示了 Eigen 基本类型的使用
****************************/

int main(int argc, char **argv) {

     int rows = 100;
     int cols = 100;

     Matrix<double, Dynamic, Dynamic> A = MatrixXd::Random(rows, cols);

     A = A*A.transpose();

     // std::cout << A << std::endl;
     Matrix<double, Dynamic, 1> b = MatrixXd::Random(rows, 1);

     Matrix<double, Dynamic, 1> x(rows, 1);

     x = A.colPivHouseholderQr().solve(b);

     std::cout << "qr: " << x.transpose() << std::endl;

     x = A.llt().solve(b);

     std::cout << "ch: " << x.transpose() << std::endl;

     return 0;
}