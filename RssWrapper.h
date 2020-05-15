#pragma once

#include <stdio.h>
#include <string>
#include <sstream>
#include "ad/rss/core/RssCheck.hpp"
#include <boost/format.hpp>

struct Lane {
//    Lane() {}
    Lane(double x, double y, double length, double width, double heading, int id): 
        x(x), y(y), length(length), width(width), heading(heading), id(id) {}
    std::string str() {
//        static std::stringstream temp;
//        temp.clear();
//        temp << "Lane(x=" << x << ", y=" << y << ", length=" << length << ", width=" << width << ", heading=" << heading << std::endl;
//        return temp.str();
//        static auto temp = boost::format("Lane(x=%f, y=%f, length=%f, width=%f, heading=%f\n") % x % y % length % width % heading;
//        std::cout << temp;
        static char buffer[1000];
        sprintf(buffer, "Lane(x=%f, y=%f, length=%f, width=%f, heading=%f, id=%d\n", x, y, length, width, heading, id);
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
    int id;
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

struct VControl {
//    VControl(double throttle, double brake, double steer):
//        throttle(throttle), brake(brake), steer(steer) {}
    VControl() {
        throttle = 0;
        brake = 0;
        steer = 0;
    }
    std::string str() {
        static char buffer[1000];
        sprintf(buffer, "VControl(throttle=%f, brake=%f, steer=%f\n", throttle, brake, steer);
        return std::string(buffer);
    }
    double throttle;
    double brake;
    double steer;
};

extern "C" int RssCheck(Lane, Vehicle, Vehicle, VControl&);

extern "C" std::string ssWorld(void);
extern "C" std::string ssSituation();
extern "C" std::string ssState();
extern "C" std::string ssResponse();
extern "C" std::string ssRestriction();
extern "C" std::string version();
