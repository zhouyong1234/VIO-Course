#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <Eigen/Core>

class MapPoint
{
private:
    Eigen::Vector3d position_;
    int id_;
    bool fixed_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MapPoint(const Eigen::Vector3d& position, int id, bool fixed = false);
    const Eigen::Vector3d& getPosition();
    void setPosition(const Eigen::Vector3d& position);
    int getId();
    void setId(int id);
    void setFixed();
    bool isFixed();
    void addDeltaPosition(const Eigen::Vector3d& delta_position);
    
    int state_index_;
};




#endif