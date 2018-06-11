//
// Created by vincent on 18-6-1.
//

#include "Cell.h"
/**
 * average height
 * @param height
 */
void Cell::UpdateHeight(double height)
{
    if (height > this->height)
    {
        this->height = height;
    }
}