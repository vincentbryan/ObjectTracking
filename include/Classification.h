//
// Created by vincent on 18-6-1.
//

#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <utility>
#include "Grids.h"
#include "Object.h"

class Classification
{
private:
    unsigned int mClusterId;
    Grids * mGrids;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mClusteredCloud;
    std::vector<Object> mResult;

    void ComponentClustering();
    void Search(int i, int j);
public:
    explicit Classification(Grids * grids, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetColoredCloud(){return mClusteredCloud;}
    std::vector<Object> GetResult(){ return mResult;}
};
