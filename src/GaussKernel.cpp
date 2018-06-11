//
// Created by vincent on 18-6-1.
//
#pragma once
#include <cassert>
#include <cmath>
#include "GaussKernel.h"

GaussKernel::GaussKernel(double meanValue, double standardDeviation, double sampleStep, int size)
{
    this->standardDeviation = standardDeviation;
    this->meanValue = meanValue;
    this->sampleStep = sampleStep;
    assert(size % 2 == 1);
    this->size = size;

    double sum = 0;
    mKernel.resize(size);

    int middle = size / 2;
    for (int i = 0; i <= size / 2; ++i) {
        double x = meanValue + i * sampleStep;
        double y =  1.0f / (sqrt(2 * M_PI)*standardDeviation) * exp(-1.0f * pow(x - meanValue, 2) / (2*standardDeviation*standardDeviation));
        if(i == 0)
        {
            mKernel[middle] = y;
            sum += y;
        }
        else
        {
            mKernel[middle + i] = mKernel[middle - i] = y;
            sum += 2*y;
        }
    }

    for (double it : mKernel) it /= sum;

}
