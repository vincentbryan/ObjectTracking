//
// Created by vincent on 18-5-31.
//
#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <utility>
#include "Cell.h"
#include "Grids.h"

class Segmentation
{
private:
    const double groundHeight = -1.2;
    const double heightDiffMax = 0.4;
    Grids * mGrids;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mGroundData;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mEvaluatedData;

public:
    explicit Segmentation(Grids * grids, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void MeanFilter();

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetGroundData(){ return mGroundData;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr GetEvaluateData(){ return mEvaluatedData;}

};



