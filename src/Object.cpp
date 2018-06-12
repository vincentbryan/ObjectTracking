//
// Created by vincent on 18-6-8.
//

#include "Object.h"

void Object::AddPosition(std::pair<int, int> pos)
{
    if(std::find(mPositions.begin(), mPositions.end(), pos) == mPositions.end())
    {
        mPositions.emplace_back(pos);
    }
}

void Object::AddPoint(pcl::PointXYZI point)
{
    mPoints->push_back(point);
}

void Object::GenerateProfile()
{
    assert(!mPoints->empty() && !mPositions.empty());

    mProfile.pointNumber = mPoints->size();
    mProfile.cellNumber = mPositions.size();

    double max_height = mPoints->front().z;
    double max_intensity = mPoints->front().intensity;
    double mean_height = 0;
    double mean_intensity = 0;
    std::pair<double, double> center = {0, 0};

    for (auto & point : *mPoints)
    {
        if (point.z > max_height) max_height = point.z;
        if (point.intensity > max_intensity) max_intensity = point.intensity;
        mean_height += point.z;
        mean_intensity += point.intensity;
        center.first += point.x;
        center.second += point.y;
    }

    center.first /= mPoints->size();
    center.second /= mPoints->size();
    mean_height /= mPoints->size();
    mean_intensity /= mPoints->size();

    mProfile.center = center;
    mProfile.maxHeight = max_height;
    mProfile.maxIntensity = max_intensity;
    mProfile.meanHeight = mean_height;
    mProfile.meanIntensity = mean_intensity;

}