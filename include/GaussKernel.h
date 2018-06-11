//
// Created by vincent on 18-6-1.
//

#pragma once

#include <vector>


class GaussKernel
{
private:
    double standardDeviation;
    double meanValue;
    double sampleStep;
    int size;

    std::vector<double> mKernel;

public:
    GaussKernel(double meanValue, double standardDeviation, double sampleStep, int size);

    std::vector<double> GetKernel(){return mKernel;}
    //TODO Convolve use template

};
