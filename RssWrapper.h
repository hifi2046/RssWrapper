#pragma once

#include "ad/rss/core/RssCheck.hpp"

struct Lane {
//    Lane() {}
    Lane(double x, double y, double length, double width, double heading): 
        x(x), y(y), length(length), width(width), heading(heading) {}
    double x;
    double y;
    double length;
    double width;
    double heading;
};

struct Vehicle {
//    Vehicle() {}
    Vehicle(double x, double y, double heading, double velocity):
        x(x), y(y), heading(heading), velocity(velocity) {}
    double x;
    double y;
    double heading;
    double velocity;
};

int RssCheck(Lane, Vehicle, Vehicle);
