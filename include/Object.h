//
// Created by vincent on 18-6-8.
//
#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
class Object
{
private:
    unsigned int mObjectId;
    unsigned int mClusterId;
    std::vector<std::pair<int, int> > mPositions;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPoints;

public:
    explicit Object(unsigned int id) : mClusterId(id){}
    void AddPosition(std::pair<int, int> pos);
    void AddPoint(pcl::PointXYZRGB point);

    class Profile
    {
    public:
        unsigned int pointNumber;
        unsigned int cellNumber;
        double meanHeight;
        double maxHeight;
        std::pair<double, double> center;

        explicit Profile()
        {

        }
    };


};


