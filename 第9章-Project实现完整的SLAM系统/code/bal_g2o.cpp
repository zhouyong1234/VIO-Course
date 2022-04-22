#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>
#include <iostream>

#include <sophus/se3.hpp>
#include "common.h"

using namespace Sophus;
using namespace Eigen;
using namespace std;


struct Pose
{
    Pose() {}
    
    explicit Pose(double *data)
    {
        rotation = SO3d::exp(Vector3d(data[0], data[1], data[2]));
        translation = Vector3d(data[3], data[4], data[5]);
        focus = data[6];
        k1 = data[7];
        k2 = data[8];
    }

    void set_to(double *data)
    {
        auto r = rotation.log();
        for(int i = 0; i < 3; ++i) data[i] = r[i];
        for(int i = 0; i < 3; ++i) data[i+3] = translation[i];
        data[6] = focus;
        data[7] = k1;
        data[8] = k2;
    }


    SO3d rotation;
    Vector3d translation = Vector3d::Zero();
    double focus = 0;
    double k1 = 0, k2 = 0;
};


class PoseVertex : public g2o::BaseVertex<9, Pose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PoseVertex() {}
    
    virtual void setToOriginImpl() override
    {
        _estimate = Pose();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate.rotation = SO3d::exp(Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Vector3d(update[3], update[4], update[5]);
        _estimate.focus += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    Vector2d project(const Vector3d &point)
    {
        Vector3d pc = _estimate.rotation * point + _estimate.translation;
        pc = -pc / pc[2];
        double r2 = pc.squaredNorm();
        double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);
        return Vector2d(_estimate.focus * distortion * pc[0], _estimate.focus * distortion * pc[1]);
    }

    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}

};


class PointVertex : public g2o::BaseVertex<3, Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PointVertex() {}
    

    virtual void setToOriginImpl() override
    {
        _estimate = Vector3d(0,0,0);
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
};


class Edge : public g2o::BaseBinaryEdge<2, Vector2d, PoseVertex, PointVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void computeError() override
    {
        auto v0 = (PoseVertex *) _vertices[0];
        auto v1 = (PointVertex *) _vertices[1];
        auto proj = v0->project(v1->estimate());
        _error = proj - _measurement;
    }

    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
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

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    const double *observations = bal_problem.observations();

    vector<PoseVertex *> pose_vertexs;
    vector<PointVertex *> point_vertexs;

    // std::cout << "num_cameras: " << bal_problem.num_cameras() << " " << "\tnum_points: " << bal_problem.num_points();
    // std::cout << "\tnum_observations: " << bal_problem.num_observations() << std::endl;
    std::cout << "***************************** Solved by G2O *****************************\n";

    for(int i = 0; i < bal_problem.num_cameras(); ++i)
    {
        PoseVertex *v = new PoseVertex();
        double *camera = cameras + camera_block_size * i;
        v->setId(i);
        v->setEstimate(Pose(camera));
        optimizer.addVertex(v);
        pose_vertexs.push_back(v);
    }

    for(int i = 0; i < bal_problem.num_points(); ++i)
    {
        PointVertex *v = new PointVertex();
        double *point = points + point_block_size * i;
        v->setId(i + bal_problem.num_cameras());
        v->setEstimate(Vector3d(point[0], point[1], point[2]));
        v->setMarginalized(true);
        v->setFixed(true);
        optimizer.addVertex(v);
        point_vertexs.push_back(v);
    }

    for(int i = 0; i < bal_problem.num_observations(); ++i)
    {
        Edge *edge = new Edge;
        edge->setVertex(0, pose_vertexs[bal_problem.camera_index()[i]]);
        edge->setVertex(1, point_vertexs[bal_problem.point_index()[i]]);

        edge->setMeasurement(Vector2d(observations[2*i+0], observations[2*i+1]));
        edge->setInformation(Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    // std::cout << "optimize start" << std::endl;
    optimizer.optimize(25);
    // std::cout << "optimize finish" << std::endl;

    double sum_rot_error = 0.0;
    double sum_trans_error = 0.0;
    for(size_t i = 0; i < bal_problem.num_cameras(); ++i)
    {
        auto vertex = pose_vertexs[i];
        auto estimate = vertex->estimate();
        const Sophus::SE3d& opt_pose = SE3d(estimate.rotation, estimate.translation);
        double *camera = cameras + camera_block_size * i;
        const Sophus::SE3d& org_pose = SE3d(SO3d::exp(Vector3d(camera[0], camera[1], camera[2])), Vector3d(camera[3], camera[4], camera[5]));
        Sophus::SE3d pose_err = opt_pose * org_pose.inverse();
        sum_rot_error += pose_err.so3().log().norm();
        sum_trans_error += pose_err.translation().norm();

    }

    std::cout << "Mean rot error: " << sum_rot_error / (double)(bal_problem.num_cameras()) << "\tMean trans error: " << sum_trans_error / (double)(bal_problem.num_cameras()) << std::endl;

    // for(int i = 0; i < bal_problem.num_cameras(); ++i)
    // {
    //     double *camera = cameras + camera_block_size * i;
    //     auto vertex = pose_vertexs[i];
    //     auto estimate = vertex->estimate();
    //     estimate.set_to(camera);
    // }

    // for(int i = 0; i < bal_problem.num_points(); ++i)
    // {
    //     double *point = points + point_block_size * i;
    //     auto vertex = point_vertexs[i];
    //     for(int k = 0; k < 3; ++k) point[k] = vertex->estimate()[k];
    // }
}





