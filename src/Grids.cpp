//
// Created by vincent on 18-6-4.
//
#include <pcl/point_types.h>
#include "Grids.h"

Grids::Grids(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double gridLength)
{
    assert(!cloud->empty());

    mGridLength = gridLength;
    x_min = x_max = cloud->front().x;
    y_min = y_max = cloud->front().y;
    for (auto & point : *cloud)
    {
        if (point.x > x_max) x_max = point.x;
        if (point.x < x_min) x_min = point.x;
        if (point.y > y_max) y_max = point.y;
        if (point.y < y_min) y_min = point.y;
    }

    mGridSize = std::pair<int, int>(
        floor((y_max-y_min) / gridLength) + 1,
        floor((x_max-x_min) / gridLength) + 1
    );

    mData.resize(mGridSize.first);
    for (auto & i : mData) i.resize(mGridSize.second);

    for (auto & point : *cloud)
    {
        auto y_index = int( floor((point.y - y_min) / mGridLength) );
        auto x_index = int( floor((point.x - x_min) / mGridLength) );
        mData[y_index][x_index].UpdateHeight(point.z);
    }

    for (auto & row : mData)
    {
//        GaussFilter(row);

        for (auto it = row.begin(); it != row.end(); ++it)
        {
            if (it == row.begin()) it->heightDiff = fabs(it->height - (it+1)->height);
            else if(it+1 == row.end()) it->heightDiff = fabs(it->height - (it-1)->height);
            else it->heightDiff = std::max(fabs(it->height - (it+1)->height) , fabs(it->height - (it-1)->height));
        }
    }
}

void Grids::GaussFilter(std::vector<Cell> & row)
{
    double mean_value = 1.5;
    double standard_deviation = 1;
    double sample_step = 1;
    int kernel_size = 3;
    GaussKernel Gauss(mean_value, standard_deviation, sample_step, kernel_size);
    std::vector<double> kernel = Gauss.GetKernel();

    std::vector<Cell> origin = row;
    for(int i = 0; i < row.size(); i++)
    {
        double sum = 0;
        int middle = kernel_size / 2;
        for(int j = 0; j < kernel.size(); j++)
        {
            int index = i + (j - middle);
            if(index < 0) index = (index + row.size()) % row.size();
            if(index >= row.size()) index %= row.size();

            sum += origin[index].height * kernel[j];
        }
        row[i].height = sum;
    }
}

std::pair<int, int> Grids::Convert(double x, double y)
{
    auto y_index = int( floor((y - y_min) / mGridLength) );
    auto x_index = int( floor((x - x_min) / mGridLength) );
    return {y_index, x_index};
}


