//
// Created by vincent on 18-5-31.
//
#include <pcl/point_types.h>
#include <GaussKernel.h>
#include "Segmentation.h"

Segmentation::Segmentation(Grids * grids, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    : mGrids(grids),
      mGroundData(new pcl::PointCloud<pcl::PointXYZI>()),
      mEvaluatedData(new pcl::PointCloud<pcl::PointXYZI>())
{
    for (auto & row : grids->mData)
    {
        for(auto & it : row)
        {
            if(it.height < groundHeight)
                it.SetType(Cell::Ground);
            else
                it.SetType(Cell::Obstacle);
        }
    }

    for(auto it = cloud->begin(); it != cloud->end(); it++)
    {
        if (it->z < groundHeight)
            mGroundData->push_back(*it);
        else
            mEvaluatedData->push_back(*it);
    }

}

void Segmentation::MeanFilter()
{
    auto AllGround = [&](Cell t1, Cell t2, Cell t3, Cell t4){
        if (t1.GetType() == Cell::Ground &&
            t2.GetType() == Cell::Ground &&
            t3.GetType() == Cell::Ground &&
            t4.GetType() == Cell::Ground)
            return true;
        else
            return false;
    };

    for (int i = 1; i < mGrids->mData.size()-1; ++i) {
        for(int j = 1; j < mGrids->mData[i].size()-1; j++)
        {
            if (!mGrids->mData[i][j].GetType() == Cell::Ground)
            {
                if(AllGround(mGrids->mData[i+1][j],
                   mGrids->mData[i-1][j],
                   mGrids->mData[i][j+1],
                   mGrids->mData[i][j-1]))
                {
                    double mean = ( mGrids->mData[j][i+1].height +
                                    mGrids->mData[j][i-1].height +
                                    mGrids->mData[j+1][i].height +
                                    mGrids->mData[j-1][i].height ) / 4.0f;
                    mGrids->mData[j][i].height = mean;
                }
            }
        }
    }
}
