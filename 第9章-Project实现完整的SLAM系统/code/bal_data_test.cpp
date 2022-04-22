#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include "common.h"
#include "bundle_adjustment.h"

using namespace Sophus;
using namespace Eigen;
using namespace std;


class Observation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Observation(int mpt_id, int cam_id, const Eigen::Vector2d& ob) :mpt_id_(mpt_id), cam_id_(cam_id), ob_(ob) {}
    
    int mpt_id_;
    int cam_id_;
    Eigen::Vector2d ob_;
};

void SolveBA(BALProblem &bal_problem);

int main(int argc, char **argv)
{

    if(argc != 2)
    {
        std::cout << "Error" << std::endl;
        return 1;
    }

    BALProblem bal_problem(argv[1]);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1, 0.5, 0.5);
    bal_problem.WriteToPLYFile("initial.ply");
    SolveBA(bal_problem);
    bal_problem.WriteToPLYFile("final.ply");

    return 0;

}
    

void SolveBA(BALProblem &bal_problem)
{

    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    double *points = bal_problem.mutable_points();
    double *cameras = bal_problem.mutable_cameras();
    const double *observations = bal_problem.observations();


    std::cout << "***************************** Solved by BA *****************************\n";


    BundleAdjustment ba;
    ba.setConvergenceCondition(100, 1e-10, 1e-15);
    ba.setVerbose(true);

    // std::cout << "num_cameras: " << bal_problem.num_cameras() << " " << "\tnum_points: " << bal_problem.num_points();
    // std::cout << "\tnum_observations: " << bal_problem.num_observations() << std::endl;

    for(int i = 0; i < bal_problem.num_points(); ++i)
    {
        double *point = points + point_block_size * i;
        const Eigen::Vector3d& npt = Vector3d(point[0], point[1], point[2]);
        MapPoint* mpt = new MapPoint(npt, i);
        mpt->setFixed();
        ba.addMapPoint(mpt);
    }

    for(int i = 0; i < bal_problem.num_cameras(); ++i)
    {
        double *camera = cameras + camera_block_size * i;
        const Sophus::SE3d& ncam = SE3d(SO3d::exp(Vector3d(camera[0], camera[1], camera[2])), Vector3d(camera[3], camera[4], camera[5]));
        Camera* cam = new Camera(ncam, i, camera[6], camera[7], camera[8]);
        ba.addCamera(cam);
    }

    
    for(int i = 0; i < bal_problem.num_observations(); ++i)
    {
        
        const Observation& ob = Observation(bal_problem.point_index()[i], bal_problem.camera_index()[i], Vector2d(observations[2*i+0], observations[2*i+1]));
        // std::cout << "point_index: " << ob.mpt_id_ << " camera_index: " << ob.cam_id_ << std::endl;
        // std::cout << "observation: " << ob.ob_.transpose() << std::endl;
        MapPoint* mpt = ba.getMapPoint(ob.mpt_id_);
        Camera* cam = ba.getCamera(ob.cam_id_);
        CostFunction* cost_func = new CostFunction(mpt, cam, ob.ob_);
        ba.addCostFunction(cost_func);
    }

    
    ba.optimize();

    double sum_rot_error = 0.0;
    double sum_trans_error = 0.0;
    for(size_t i = 0; i < bal_problem.num_cameras(); ++i)
    {
        Camera* cam = ba.getCamera(i);
        const Sophus::SE3d& opt_pose = cam->getPose();
        double *camera = cameras + camera_block_size * i;
        const Sophus::SE3d& org_pose = SE3d(SO3d::exp(Vector3d(camera[0], camera[1], camera[2])), Vector3d(camera[3], camera[4], camera[5]));
        Sophus::SE3d pose_err = opt_pose * org_pose.inverse();
        sum_rot_error += pose_err.so3().log().norm();
        sum_trans_error += pose_err.translation().norm();

    }

    std::cout << "Mean rot error: " << sum_rot_error / (double)(bal_problem.num_cameras()) << "\tMean trans error: " << sum_trans_error / (double)(bal_problem.num_cameras()) << std::endl;

}


