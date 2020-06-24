#pragma once

#include <stdio.h>
#include <string>
#include <sstream>
#include "ad/rss/core/RssCheck.hpp"
#include "ad/rss/world/AccelerationRestriction.hpp"
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
        x(x), y(y), heading(heading), velocity(velocity) {
            acceleration = 0;
            speedLimit = 200.0 / 3.6;
            slope = 0;
        }
    std::string str() {
//        std::stringstream temp;
//        temp << "Vehicle(x=" << x << ", y=" << y << ", heading=" << heading << ", velocity=" << velocity << std::endl;
//        return temp.str();
        static char buffer[1000];
        sprintf(buffer, "Vehicle(x=%f, y=%f, heading=%f, velocity=%f, acceleration=%f, speedLimit=%f, slope=%fn", x, y, heading, velocity, acceleration, speedLimit, slope);
        return std::string(buffer);
    }
    double x;
    double y;
    double heading;
    double velocity;
    double acceleration;
    double speedLimit;
    double slope;
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

struct Restriction {
    Restriction() {
        front = 0;
        left = 0;
        right = 0;
    }
    std::string str() {
        static char buffer[1000];
        sprintf(buffer, "Restriction(front=%f, left=%f, right=%f\n", front, left, right);
        return std::string(buffer);
    }
    double front;
    double left;
    double right;
};

struct Setting {
    Setting() {
        style = 0;
        weather = 0;
        surface = 0;
    }
    std::string str() {
        static char buffer[1000];
        sprintf(buffer, "Setting(style=%d, weather=%d, surface=%d, friction=%f\n", style, weather, surface, friction());
        return std::string(buffer);
    }
    double friction() {
        switch(weather) {
            case 7:
            case 14:
                return 0.8;
            case 5:
            case 12:
                return 0.7;
            case 6:
            case 13:
                return 0.5;
            default:
                return 1;
        }
        return 1;
    }
    int style;
    int weather;
    int surface;
};

struct Dynamics {
    Dynamics() {
        accelMax = 3.5;
        brakeMax = -8;
        brakeMin = -4;
        brakeMinCorrect = -3;
        accelMaxLat = 0.2;
        brakeMinLat = -0.8;
        responseTime = 1;
        brakeLeaving = -2;
        brakeFollowing = -3;
        brakeApproaching = -4;
        k = -4;
    }
    std::string str() {
        static char buffer[1000];
        sprintf(buffer, "Dynamics(accelMax=%f, brakeMax=%f, brakeMin=%f, brakeMinCorrect=%f, accelMaxLat=%f, brakeMinLat=%f, responseTime=%f, brakeLeaving=%f, brakeFollowing=%f, brakeApproaching=%f, k=%f\n", accelMax, brakeMax, brakeMin, brakeMinCorrect, accelMaxLat, brakeMinLat, responseTime, brakeLeaving, brakeFollowing, brakeApproaching, k);
        return std::string(buffer);
    }
    double accelMax;
    double brakeMax;
    double brakeMin;
    double brakeMinCorrect;
    double accelMaxLat;
    double brakeMinLat;
    double responseTime;
    double brakeLeaving;
    double brakeFollowing;
    double brakeApproaching;
    double k;
};

extern "C" int RssCheck(Lane, Vehicle, Vehicle, Restriction&);
extern "C" int RssRestrict(Restriction, VControl&);
extern "C" int RssSet(Dynamics);
extern "C" int RssScenario(Setting);

extern "C" std::string ssWorld(void);
extern "C" std::string ssSituation();
extern "C" std::string ssState();
extern "C" std::string ssResponse();
extern "C" std::string ssRestriction();
extern "C" std::string version();
