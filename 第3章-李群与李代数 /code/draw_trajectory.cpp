#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;

// path to trajectory file
string trajectory_file = "/home/touchair/dev_ws/src/code/trajectory.txt";

string groundtruth_file = "/home/touchair/dev_ws/src/code/groundtruth.txt";

string estimated_file = "/home/touchair/dev_ws/src/code/estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> >);
// void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses, vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses);

double CalRMSE(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses, vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses);

int main(int argc, char **argv) {

    // vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > poses;

    // vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > ground_truth_poses;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > estimated_poses;

    std::fstream ground_truth(groundtruth_file, std::fstream::in);
    std::fstream estimated(estimated_file, std::fstream::in);

    /// implement pose reading code
    // start your code here (5~10 lines)

    // std::fstream trajectory(trajectory_file, std::fstream::in);

    double t, tx, ty, tz, qx, qy, qz, qw;

    // while (!trajectory.eof())
    // {
    //     trajectory >> t;
    //     trajectory >> tx;
    //     trajectory >> ty;
    //     trajectory >> tz;
    //     trajectory >> qx;
    //     trajectory >> qy;
    //     trajectory >> qz;
    //     trajectory >> qw;
    //     Eigen::Quaterniond q(qw, qx, qy, qz);
    //     q.normalize();
    //     poses.push_back(Sophus::SE3d(q, Eigen::Vector3d(tx, ty, tz)));
    // }

    // trajectory.close();

    while (!ground_truth.eof())
    {
        ground_truth >> t;
        ground_truth >> tx;
        ground_truth >> ty;
        ground_truth >> tz;
        ground_truth >> qx;
        ground_truth >> qy;
        ground_truth >> qz;
        ground_truth >> qw;
        ground_truth_poses.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
    }

    ground_truth.close();

    while (!estimated.eof())
    {
        estimated >> t;
        estimated >> tx;
        estimated >> ty;
        estimated >> tz;
        estimated >> qx;
        estimated >> qy;
        estimated >> qz;
        estimated >> qw;
        estimated_poses.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
    }

    estimated.close();


    cout << "rmse: " << CalRMSE(ground_truth_poses, estimated_poses) << endl;


    DrawTrajectory(ground_truth_poses, estimated_poses);

    // end your code here

    // draw trajectory in pangolin
    // DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > poses){

// void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

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