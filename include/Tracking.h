//
// Created by vincent on 18-6-8.
//

#pragma once

#include "Object.h"
#include <vector>

class Tracking
{
private:
    std::vector<Object::Profile> mProfiles;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mColoredObjects;

    static size_t sObjectId;
    const double cSimilarity = 120;

public:
    Tracking();

    Tracking(std::vector<Object> objects);

    void Matching(std::vector<Object> objects);

    double Compare(Object::Profile p1, Object::Profile p2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetColoredObjects();

};


