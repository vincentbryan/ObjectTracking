//
// Created by vincent on 18-5-30.
//

#pragma once

class Cell
{
public:
    enum Type{
        Undefined, Initialized, Ground, Obstacle, Clustered
    };
    double height;
    double heightDiff;
    unsigned int clusterId;
    Type type;

    Cell() : height(-10000.0f), heightDiff(0), clusterId(0), type(Cell::Undefined){};

    void UpdateHeight(double height);
    void SetType(Cell::Type type){this->type = type;}
    Type GetType(){ return type;}
};
