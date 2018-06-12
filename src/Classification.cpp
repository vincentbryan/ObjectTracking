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

    for (unsigned int i = 1; i <= mClusterId; ++i)
        mResult.emplace_back(Object(i));

    assert(mClusterId > 0);

    for (auto & point : *cloud)
    {
        auto index = mGrids->Convert(point.x, point.y);
        unsigned int cluster_id = mGrids->mData[index.first][index.second].clusterId;

        mClusteredCloud->push_back(point);

        mResult[cluster_id-1].AddPoint(point);
        mResult[cluster_id-1].AddPosition(index);
    }

    for (auto & result : mResult) result.GenerateProfile();

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
                mClusterId++;
                search(i, j);
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
