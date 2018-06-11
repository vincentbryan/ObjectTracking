//
// Created by vincent on 18-6-4.
//
#pragma once
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <utility>
#include "GaussKernel.h"
#include "Cell.h"

class Grids
{
public:
    const double heightMax = -0.4;
    const double heightMin = -2.0;
    const double heightSensor = 2;

    double x_min, x_max, y_min, y_max ;
    double mGridLength;

    std::pair<unsigned long, unsigned long> mGridSize;
    std::vector<std::vector<Cell> > mData;
public:
    explicit Grids(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double gridLength);
    void GaussFilter(std::vector<Cell> & row);
    std::pair<int, int> Convert(double x, double y);
};


