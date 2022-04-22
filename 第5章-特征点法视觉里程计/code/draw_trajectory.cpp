#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// path to trajectory file
std::string compare_file = "/home/touchair/下载/视觉SLAM课程/L5/code/compare.txt";

void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses, vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses);

double CalRMSE(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses, vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses);

int main(int argc, char **argv) {


    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses;

    std::fstream compare(compare_file, std::fstream::in);


    double time, tx, ty, tz, qx, qy, qz, qw;

    while (!compare.eof())
    {
        compare >> time;
        compare >> tx;
        compare >> ty;
        compare >> tz;
        compare >> qx;
        compare >> qy;
        compare >> qz;
        compare >> qw;
        estimated_poses.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));

        compare >> time;
        compare >> tx;
        compare >> ty;
        compare >> tz;
        compare >> qx;
        compare >> qy;
        compare >> qz;
        compare >> qw;
        ground_truth_poses.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));

    }

    compare.close();

/*****************************************************************************************************************
******************************************************before******************************************************
******************************************************************************************************************/

    cout << "rmse: " << CalRMSE(ground_truth_poses, estimated_poses) << endl;
    // DrawTrajectory(ground_truth_poses, estimated_poses);


/*****************************************************************************************************************
******************************************************after*******************************************************
******************************************************************************************************************/

    Eigen::Vector3d center_es(0,0,0);
    Eigen::Vector3d center_gt(0,0,0);

    for(int i = 0; i < ground_truth_poses.size(); i++)
    {
        center_es += estimated_poses[i].translation();
        center_gt += ground_truth_poses[i].translation();
    }

    center_es /= ground_truth_poses.size();
    center_gt /= ground_truth_poses.size();

    std::vector<Eigen::Vector3d> t_es;
    std::vector<Eigen::Vector3d> t_gt;
    for(int i = 0; i < ground_truth_poses.size(); i++)
    {
        t_es.push_back(estimated_poses[i].translation() - center_es);
        t_gt.push_back(ground_truth_poses[i].translation() - center_gt);
    }

    Eigen::MatrixXd R;
    Eigen::VectorXd t;
    Eigen::Matrix3d W;
    W.setZero();

    for(int i = 0; i < ground_truth_poses.size(); i++)
    {
        W += t_gt[i] * t_es[i].transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU|Eigen::ComputeThinV);

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    R = U * V.transpose();

    // std::cout << "R: " << std::endl << R << std::endl;

    t = center_gt - R * center_es;

    // std::cout << "t: " << t.transpose() << std::endl;

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > truth_poses;

    R = R.inverse();
    t = -R * t;

    for(int i = 0; i < ground_truth_poses.size(); i++)
    {
        Sophus::SE3d p_rt;
        p_rt = Sophus::SE3d(R, t) * ground_truth_poses[i];
        truth_poses.push_back(p_rt);
    }

    cout << "rmse: " << CalRMSE(truth_poses, estimated_poses) << endl;
    DrawTrajectory(truth_poses, estimated_poses);

    return 0;
}

/*******************************************************************************************/

void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses, vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses)
{
    if (ground_truth_poses.empty() || estimated_poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < ground_truth_poses.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = ground_truth_poses[i], p2 = ground_truth_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            p1 = estimated_poses[i], p2 = estimated_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

double CalRMSE(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses, vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses)
{
    double rmse = 0.0;
    for(int i = 0; i < ground_truth_poses.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> se3;
        se3 = (ground_truth_poses[i].inverse() * estimated_poses[i]).log();
        rmse += se3.squaredNorm();
    }

    rmse = sqrt(rmse / ground_truth_poses.size());

    return rmse;
}