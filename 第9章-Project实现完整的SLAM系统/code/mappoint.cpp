#include "mappoint.h"

MapPoint::MapPoint(const Eigen::Vector3d& position, int id, bool fixed) :position_(position), id_(id), fixed_(fixed), state_index_(-1) {}

const Eigen::Vector3d& MapPoint::getPosition()
{
    return position_;
}

void MapPoint::setPosition(const Eigen::Vector3d& position)
{
    position_ = position;
}

int MapPoint::getId()
{
    return id_;
}

void MapPoint::setId(int id)
{
    id_ = id;
}

void MapPoint::setFixed()
{
    fixed_ = true;
}

bool MapPoint::isFixed()
{
    return fixed_;
}

void MapPoint::addDeltaPosition(const Eigen::Vector3d& delta_position)
{
    position_ += delta_position;
}