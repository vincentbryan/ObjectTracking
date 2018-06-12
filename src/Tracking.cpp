//
// Created by vincent on 18-6-8.
//

#include <ros/assert.h>
#include <Object.h>
#include "Tracking.h"

size_t Tracking::sObjectId = 0;

Tracking::Tracking() : mColoredObjects(new pcl::PointCloud<pcl::PointXYZRGB>()){}

Tracking::Tracking(std::vector<Object> objects)
{
    for (auto & obj : objects)
    {
        obj.SetObjectId( ++sObjectId );
        mProfiles.emplace_back(obj.GetProfile());
    }
}

void Tracking::Matching(std::vector<Object> objects)
{
    mColoredObjects->clear();
    std::vector<Object::Profile> new_profiles;

    for (auto &  profile : mProfiles)
    {
        double min_score = std::numeric_limits<double>::max();
        int index = -1;

        for (int i = 0, size = objects.size() ; i < size; ++i)
        {
            if (!objects[i].IsMatched())
            {
                double score = Compare(profile, objects[i].GetProfile());
                if(score < cSimilarity && score < min_score)
                {
                    index = i;
                    min_score = score;
                }
            }
        }

        if(index != -1)
        {
            objects[index].SetObjectId(profile.objectId);
            objects[index].Matched();
            new_profiles.emplace_back(objects[index].GetProfile());

            size_t id = profile.objectId;
            for (auto point : *objects[index].GetPointCloud())
            {
                pcl::PointXYZRGB point_xyzrgb(uint8_t((500*id)%255),
                                              uint8_t((100*id)%255),
                                              uint8_t((150*id)%255));
                point_xyzrgb.x = point.x;
                point_xyzrgb.y = point.y;
                point_xyzrgb.z = point.z;

                mColoredObjects->push_back(point_xyzrgb);
            }

        }
        else
        {
            profile.Elapse();
            if(profile.IsAlive()) new_profiles.emplace_back(profile);
        }

    }

    for(auto & target : objects)
    {
        if(!target.IsMatched())
        {
            target.SetObjectId(++sObjectId);
            new_profiles.emplace_back(target.GetProfile());

            size_t id = target.GetProfile().objectId;
            for (auto point : *target.GetPointCloud())
            {
                pcl::PointXYZRGB point_xyzrgb(uint8_t((500*id)%255),
                                              uint8_t((100*id)%255),
                                              uint8_t((150*id)%255));
                point_xyzrgb.x = point.x;
                point_xyzrgb.y = point.y;
                point_xyzrgb.z = point.z;

                mColoredObjects->push_back(point_xyzrgb);
            }

        }
    }

    mProfiles = new_profiles;
}

double Tracking::Compare(Object::Profile p1, Object::Profile p2)
{
    double score = 0;

    score = abs(p1.pointNumber - p2.pointNumber) +
            abs(p1.cellNumber - p2.cellNumber) +
            abs(p1.maxHeight - p2.maxHeight) +
            abs(p1.meanHeight - p2.meanHeight) +
            abs(p1.maxIntensity - p2.maxIntensity) +
            abs(p1.meanIntensity - p2.meanIntensity);

    return score;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tracking::GetColoredObjects()
{
    return mColoredObjects;
}
