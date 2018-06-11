//
// Created by vincent on 18-6-1.
//
#include <pcl/point_types.h>
#include "Classification.h"

Classification::Classification(Grids * grids, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    : mGrids(grids),
      mClusterId(0),
      mClusteredCloud(new pcl::PointCloud<pcl::PointXYZI>())
{
    ComponentClustering();

    for (unsigned int i = 0; i <= mClusterId; ++i)
        mResult.emplace_back(Object(i));

    assert(mClusterId > 0);

    for (auto & point : *cloud)
    {
        auto index = mGrids->Convert(point.x, point.y);
        unsigned int cluster_id = mGrids->mData[index.first][index.second].clusterId;
//        pcl::PointXYZRGB obj(uint8_t((500*cluster_id)%255),
//                             uint8_t((100*cluster_id)%255),
//                             uint8_t((150*cluster_id)%255));
        pcl::PointXYZI obj;
        obj.x = point.x;
        obj.y = point.y;
        obj.z = point.z;

        mClusteredCloud->push_back(obj);

        mResult[cluster_id].AddPoint(obj);
        mResult[cluster_id].AddPosition(index);
    }
}

void Classification::ComponentClustering()
{
    auto search = std::function<void(int, int)>();
    search = [&](int i, int j)
    {
        mGrids->mData[i][j].clusterId = mClusterId;
        mGrids->mData[i][j].type = Cell::Clustered;

        for(int n = -1; n <= 1; n++)
        {
            for(int m = -1; m <= 1; m++)
            {
                int y_index = i + n;
                int x_index = j + m;
                if( y_index >= 0 && y_index < mGrids->mGridSize.first &&
                    x_index >= 0 && x_index < mGrids->mGridSize.second &&
                    mGrids->mData[y_index][x_index].type == Cell::Obstacle)
                {
                    search(y_index, x_index);
                }
            }
        }
    };

    for (int i = 0; i < mGrids->mGridSize.first; i++)
    {
        for(int j = 0; j < mGrids->mGridSize.second; j++)
        {
            if(mGrids->mData[i][j].GetType() == Cell::Obstacle)
            {
                search(i, j);
                mClusterId++;
            }
        }
    }
}

void Classification::Search(int i, int j)
{
    mGrids->mData[i][j].clusterId = mClusterId;
    mGrids->mData[i][j].type = Cell::Clustered;

    for(int n = -1; n <= 1; n++)
    {
        for(int m = -1; m <= 1; m++)
        {
            int y_index = i + n;
            int x_index = j + m;
            if( y_index >= 0 && y_index < mGrids->mGridSize.first &&
                x_index >= 0 && x_index < mGrids->mGridSize.second &&
                mGrids->mData[y_index][x_index].type == Cell::Obstacle)
            {
                Search(y_index, x_index);
            }
        }
    }
}
