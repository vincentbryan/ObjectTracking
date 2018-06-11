//
// Created by vincent on 18-6-8.
//

#include "Object.h"

void Object::AddPosition(std::pair<int, int> pos)
{
    if(find(mPositions.begin(), mPositions.end(), pos) != mPositions.end())
    {
        mPositions.emplace_back(pos);
    }
}

void Object::AddPoint(pcl::PointXYZI point)
{
    mPoints->push_back(point);
}