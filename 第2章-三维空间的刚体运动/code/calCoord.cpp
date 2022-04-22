#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{

    Quaterniond qWR(0.55, 0.3, 0.2, 0.2);
    Vector3d tWR(0.1, 0.2, 0.3);
    qWR.normalize();

    Quaterniond qRB(0.99, 0, 0, 0.01);
    Vector3d tRB(0.05, 0, 0.5);
    qRB.normalize();

    Quaterniond qBL(0.3, 0.5, 0, 20.1);
    Vector3d tBL(0.4, 0, 0.5);
    qBL.normalize();

    Quaterniond qBC(0.8, 0.2, 0.1, 0.1);
    Vector3d tBC(0.5, 0.1, 0.5);
    qBC.normalize();

    Vector3d p0(0.3, 0.2, 1.2);

    Vector4d p0a;
    p0a << p0, 1;

    Matrix4d T_WR;
    T_WR << qWR.toRotationMatrix(), tWR,
            0, 0, 0, 1;

    Matrix4d T_RB;
    T_RB << qRB.toRotationMatrix(), tRB,
            0, 0, 0, 1;

    Matrix4d T_BL;
    T_BL << qBL.toRotationMatrix(), tBL,
            0, 0, 0, 1;

    Matrix4d T_BC;
    T_BC << qBC.toRotationMatrix(), tBC,
            0, 0, 0, 1;

    Vector4d p1a;
    p1a = T_BL.inverse() * T_BC * p0a;

    Vector3d p1;
    p1 = p1a.block<3,1>(0,0);

    Vector4d p2a;
    p2a = T_WR * T_RB * T_BL * p1a;

    Vector3d p2;
    p2 = p2a.block<3,1>(0,0);

    cout << "这个点在激光系下的坐标: " << p1.transpose() << endl;
    cout << "这个点在世界系下的坐标: " << p2.transpose() << endl;

    return 0;
}