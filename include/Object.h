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
    size_t mObjectId;
    size_t mClusterId;
    std::vector<std::pair<int, int> > mPositions;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mPoints;

public:
    explicit Object(unsigned int id)
        : mClusterId(id),
          mObjectId(0),
          mPoints(new pcl::PointCloud<pcl::PointXYZI>)
    {
        mPositions.clear();
    };

    void AddPosition(std::pair<int, int> pos);
    void AddPoint(pcl::PointXYZI point);

    void Matched()
    {
        mProfile.Matched();
    }

    bool IsMatched()
    {
        return mProfile.isMatched;
    }

    void Elapse()
    {
        mProfile.Elapse();
    }

    bool IsAlive()
    {
        return mProfile.IsAlive();
    }

    void SetObjectId(size_t id)
    {
        mObjectId = id;
        mProfile.objectId = id;
    }

    class Profile
    {
    public:
        size_t lifeTime;
        bool isMatched;

        size_t objectId;
        size_t pointNumber;
        size_t cellNumber;
        double meanHeight;
        double maxHeight;
        double meanIntensity;
        double maxIntensity;
        std::pair<double, double> center;

        explicit Profile()
        {
            lifeTime = 20;
            isMatched = false;
            objectId = 0;
            pointNumber = cellNumber = 0;
            meanHeight = maxHeight = 0;
            meanIntensity = maxIntensity = 0;
            center = {0, 0};
        }

        void Elapse()
        {
            lifeTime--;
        }

        bool IsAlive()
        {
            return lifeTime > 0;
        }

        void Matched()
        {
            lifeTime = 20;
            isMatched = true;
        }

    } mProfile;

    void GenerateProfile();

    Profile GetProfile()
    {
        return mProfile;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetPointCloud()
    {
        return mPoints;
    }
};


