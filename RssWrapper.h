#pragma once

#include <stdio.h>
#include <string>
#include <sstream>
#include "ad/rss/core/RssCheck.hpp"
#include <boost/format.hpp>

struct Lane {
//    Lane() {}
    Lane(double x, double y, double length, double width, double heading): 
        x(x), y(y), length(length), width(width), heading(heading) {}
    std::string str() {
//        static std::stringstream temp;
//        temp.clear();
//        temp << "Lane(x=" << x << ", y=" << y << ", length=" << length << ", width=" << width << ", heading=" << heading << std::endl;
//        return temp.str();
//        static auto temp = boost::format("Lane(x=%f, y=%f, length=%f, width=%f, heading=%f\n") % x % y % length % width % heading;
//        std::cout << temp;
        static char buffer[1000];
        sprintf(buffer, "Lane(x=%f, y=%f, length=%f, width=%f, heading=%f\n", x, y, length, width, heading);
        return std::string(buffer);
    }
//    int sum() {
//        return x+y+length+width+heading;
//    }
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
    std::string str() {
//        std::stringstream temp;
//        temp << "Vehicle(x=" << x << ", y=" << y << ", heading=" << heading << ", velocity=" << velocity << std::endl;
//        return temp.str();
        static char buffer[1000];
        sprintf(buffer, "Vehicle(x=%f, y=%f, heading=%f, velocity=%f\n", x, y, heading, velocity);
        return std::string(buffer);
    }
    double x;
    double y;
    double heading;
    double velocity;
};

int RssCheck(Lane, Vehicle, Vehicle);
