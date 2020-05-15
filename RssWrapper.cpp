//#include "ad/rss/core/RssResponseResolving.hpp"
//#include "ad/rss/core/RssResponseTransformation.hpp"
//#include "ad/rss/core/RssSituationChecking.hpp"
//#include "ad/rss/core/RssSituationExtraction.hpp"
//#include "ad/rss/state/RssStateOperation.hpp"
#include "ad/rss/core/RssCheck.hpp"
#include "RssWrapper.h"
#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <math.h>
#define PI 3.14159265
using namespace boost::python;

//std::stringstream sWorld;
//std::stringstream sSituation;
//std::stringstream sState;
//std::stringstream sResponse;
//std::stringstream sRestriction;
static char sWorld[10000];
//static string sWorld;
static char sSituation[10000];
static char sState[10000];
static char sResponse[10000];
static char sRestriction[10000];

void calculateOccupiedRegions(::ad::rss::world::OccupiedRegionVector &bounds, Lane &lane, Vehicle &v, float xx, float yy) {
    ::ad::rss::world::OccupiedRegion r;
    double s, t, theta, cost, sint, x, y, x0, y0, ss, tt;
    theta = lane.heading;
    cost = cos((theta - 90)*PI/180.0);
    sint = sin((theta - 90)*PI/180.0);
    x = v.x;
    y = v.y;
    x0 = lane.x;
    y0 = lane.y;
    s = (y - y0)*cost - (x - x0)*sint;
    t = (x - x0)*cost + (y - y0)*sint;
    ss = s / lane.length;
    tt = t / lane.width + 0.5;
    r.segmentId = 43;
    r.lonRange.minimum=::ad::physics::ParametricValue(ss);
    r.lonRange.maximum=::ad::physics::ParametricValue(ss);
    r.latRange.minimum=::ad::physics::ParametricValue(tt);
    r.latRange.maximum=::ad::physics::ParametricValue(tt);
//    r.lonRange.minimum=::ad::physics::ParametricValue(s);
//    r.lonRange.maximum=::ad::physics::ParametricValue(s);
//    r.latRange.minimum=::ad::physics::ParametricValue(t);
//    r.latRange.maximum=::ad::physics::ParametricValue(t);
    bounds.push_back(r);
}

void calculateLatLonVelocities(::ad::rss::world::Velocity &velocity, Lane &lane, Vehicle &v) {
    double angle = (v.heading - lane.heading)*PI/180.0;
    double vlon = std::abs(v.velocity*cos(angle));
    double vlat = std::abs(v.velocity*sin(angle));
//    velocity.speedLonMin = ::ad::physics::Speed(13);
//    velocity.speedLonMax = ::ad::physics::Speed(13);
//    velocity.speedLatMin = ::ad::physics::Speed(0);
//    velocity.speedLatMax = ::ad::physics::Speed(0);
    velocity.speedLonMin = ::ad::physics::Speed(vlon);
    velocity.speedLonMax = ::ad::physics::Speed(vlon);
    velocity.speedLatMin = ::ad::physics::Speed(vlat);
    velocity.speedLatMax = ::ad::physics::Speed(vlat);
}

int RssCheck(Lane lane, Vehicle ego, Vehicle other, VControl &control) {
    ::ad::rss::core::RssCheck rssCheck;
    ::ad::rss::situation::SituationSnapshot situationSnapshot;
    ::ad::rss::state::RssStateSnapshot rssStateSnapshot;
    ::ad::rss::state::ProperResponse properResponse;
    ::ad::rss::world::WorldModel worldModel;
    ::ad::rss::world::AccelerationRestriction accelerationRestriction;
    
    if( lane.length < 5 ) lane.length = 10;
    if( lane.width < 5 ) lane.width = 5;
    lane.length *= 2;
    
    // 设置车辆动力学参数
    worldModel.timeIndex = 16;
    worldModel.defaultEgoVehicleRssDynamics = ::ad::rss::world::RssDynamics();
    worldModel.defaultEgoVehicleRssDynamics.alphaLon.accelMax = ::ad::physics::Acceleration(3.5);
    worldModel.defaultEgoVehicleRssDynamics.alphaLon.brakeMax = ::ad::physics::Acceleration(-8.);
    worldModel.defaultEgoVehicleRssDynamics.alphaLon.brakeMin = ::ad::physics::Acceleration(-4.);
    worldModel.defaultEgoVehicleRssDynamics.alphaLon.brakeMinCorrect = ::ad::physics::Acceleration(-3);
    worldModel.defaultEgoVehicleRssDynamics.alphaLat.accelMax = ::ad::physics::Acceleration(0.2);
    worldModel.defaultEgoVehicleRssDynamics.alphaLat.brakeMin = ::ad::physics::Acceleration(-0.8);
    worldModel.defaultEgoVehicleRssDynamics.responseTime = ::ad::physics::Duration(1.);
    
    // 计算车辆位姿数据
    ::ad::rss::world::Object egoVehicle;
    ::ad::rss::world::Object otherVehicle;
    egoVehicle.objectId=23;
    egoVehicle.objectType=::ad::rss::world::ObjectType::EgoVehicle;
    calculateOccupiedRegions( egoVehicle.occupiedRegions, lane, ego, 0.1, 0.3 );
    calculateLatLonVelocities( egoVehicle.velocity, lane, ego );

    otherVehicle.objectId=24;
    otherVehicle.objectType=::ad::rss::world::ObjectType::OtherVehicle;
    calculateOccupiedRegions( otherVehicle.occupiedRegions, lane, other, 0.5, 0.3 );
    calculateLatLonVelocities( otherVehicle.velocity, lane, other );

    // 计算道路数据
    ::ad::rss::world::RoadArea roadArea;
    ::ad::rss::world::RoadSegment roadSegment;
    ::ad::rss::world::LaneSegment laneSegment;
    int length = lane.length;
    int width = lane.width;
//    uint64_t lid;
    laneSegment.id = lane.id;
    if ( laneSegment.id > 0 )
        laneSegment.drivingDirection = ::ad::rss::world::LaneDrivingDirection::Negative;
    else
        laneSegment.drivingDirection = ::ad::rss::world::LaneDrivingDirection::Positive;
    laneSegment.length.minimum = length;
    laneSegment.length.maximum = length;
    laneSegment.width.minimum = width;
    laneSegment.width.maximum = width;
//    lid = laneSegment.id;
//    if ( lid < 0 ) lid += 0xffff;
//    laneSegment.id = ((uint64_t)43 << 32) + ((uint64_t)1 << 16) + lid;
    roadSegment.push_back(laneSegment);
    roadArea.push_back(roadSegment);
        
    // 填写场景数组
    ::ad::rss::world::Scene scene;
    scene.situationType = ::ad::rss::situation::SituationType::SameDirection;
    scene.object = otherVehicle;
    scene.objectRssDynamics = worldModel.defaultEgoVehicleRssDynamics;
    scene.egoVehicle = egoVehicle;
    scene.egoVehicleRssDynamics = worldModel.defaultEgoVehicleRssDynamics;
    scene.egoVehicleRoad = roadArea;
    worldModel.scenes.push_back(scene);
    
    std::string temp, tt;
    // 查看输入适配
//    std::cout << "start world log" << std::endl;
//    auto& dyn = worldModel.defaultEgoVehicleRssDynamics;
    for( auto& x : worldModel.scenes) {
        tt = std::to_string(x.situationType);
        temp += "\n  - situationType:" + tt.substr(tt.rfind(':')+1);
        auto& v = x.egoVehicle;
        temp += "\n    egoVehicle: {objectId:" + std::to_string(v.objectId);
        tt = std::to_string(v.objectType);
        temp += ", objectType:" + tt.substr(tt.rfind(':')+1);
        auto& s = v.velocity;
        temp += ", velocity: {lon: " + std::to_string(s.speedLonMin) + "~" + std::to_string(s.speedLonMax);
        temp += ", lat: " + std::to_string(s.speedLatMin) + "~" + std::to_string(s.speedLatMax) + "}}";
        for( auto& r : v.occupiedRegions) {
            temp += "\n      - occupiedRegion: {segmentId:" + std::to_string(r.segmentId);
            temp += ", lonRange: " + std::to_string(r.lonRange.minimum) + " ~ " + std::to_string(r.lonRange.maximum);
            temp += ", latRange: " + std::to_string(r.latRange.minimum) + " ~ " + std::to_string(r.latRange.maximum) + "}";
        }
        auto& v2 = x.object;
        temp += "\n    object: {objectId:" + std::to_string(v2.objectId);
        tt = std::to_string(v2.objectType);
        temp += ", objectType:" + tt.substr(tt.rfind(':')+1);
        s = v2.velocity;
        temp += ", velocity: {lon: " + std::to_string(s.speedLonMin) + "~" + std::to_string(s.speedLonMax);
        temp += ", lat: " + std::to_string(s.speedLatMin) + "~" + std::to_string(s.speedLatMax) + "}}";
        for( auto& r : v2.occupiedRegions) {
            temp += "\n      - occupiedRegion: {segmentId:" + std::to_string(r.segmentId);
            temp += ", lonRange: " + std::to_string(r.lonRange.minimum) + " ~ " + std::to_string(r.lonRange.maximum);
            temp += ", latRange: " + std::to_string(r.latRange.minimum) + " ~ " + std::to_string(r.latRange.maximum) + "}";
        }
//        temp += "\n    objectRssDynamics" + std::to_string(x.objectRssDynamics);
    }
    sprintf( sWorld, "world: {timeIndex:%ld, size:%ld}%s", worldModel.timeIndex, worldModel.scenes.size(), temp.c_str());
//    std::cout << sWorld << std::endl;

    // 调用RSS检查主函数
//    std::cout << "start check" << std::endl;
    rssCheck.calculateAccelerationRestriction(worldModel, situationSnapshot, rssStateSnapshot, properResponse, accelerationRestriction);
//    std::cout << "end check" << std::endl;

    // 将安全限制转化为控制参数
    control.throttle = -1;
    control.brake = -1;
    control.steer = -1;
    ::ad::physics::Acceleration zeroAccel(0.0);
    if( accelerationRestriction.longitudinalRange.maximum < zeroAccel ) {
        control.throttle = 0;
        double sumBrakeTorque = 1370 * std::fabs(static_cast<double>(accelerationRestriction.longitudinalRange.minimum)) * 36 /100.0;
        control.brake = std::min(static_cast<float>(sumBrakeTorque / 6000.0), 1.0f);
    }
    ::ad::rss::world::Velocity egoVelocity = egoVehicle.velocity;
    if( accelerationRestriction.lateralLeftRange.maximum <= ::ad::physics::Acceleration(0.0)) {
        if (egoVelocity.speedLatMax < ::ad::physics::Speed(0.0)) {
          // driving to the left
          if (egoVelocity.speedLonMax != ::ad::physics::Speed(0.0)) {
            double angle = std::atan(egoVelocity.speedLatMax / egoVelocity.speedLonMax);
            control.steer = -1.f * static_cast<float>(angle);
          }
        }
    }
    if (accelerationRestriction.lateralRightRange.maximum <= ::ad::physics::Acceleration(0.0)) {
        if (egoVelocity.speedLatMax > ::ad::physics::Speed(0.0)) {
          // driving to the right
          if (egoVelocity.speedLonMax != ::ad::physics::Speed(0.0)) {
            double angle = std::atan(egoVelocity.speedLatMax / egoVelocity.speedLonMax);
            control.steer = -1.f * static_cast<float>(angle);
          }
        }
    }

//    sWorld.clear();
//    sSituation.clear();
//    sState.clear();
//    sResponse.clear();
//    sRestriction.clear();
    
//    sWorld << worldModel << std::endl;
//    std::cout << worldModel << std::endl;

    
//    std::cout << "start situation log" << std::endl;
    // 查看状况中间结果
//    sSituation << "\033[31msituationSnapshot\033[0m: timeIndex: " << situationSnapshot.timeIndex;
//    sSituation << ", size: " << situationSnapshot.situations.size() << std::endl;
    int count = 0;
    temp = "";
    for( auto &t : situationSnapshot.situations ) {
        temp += "  - objectId:" + std::to_string(t.objectId);
        temp += ", situationId:" + std::to_string(t.situationId);
        tt = std::to_string(t.situationType);
        temp += ", situationType:" + tt.substr(tt.rfind(':')+1);
        auto& x = t.relativePosition;
        tt = std::to_string(x.longitudinalPosition);
        temp += "\n    relativePosition:{lonPos:" + tt.substr(tt.rfind(':')+1);
        temp += ", lonDis:" + std::to_string((double)x.longitudinalDistance);
        tt = std::to_string(x.lateralPosition);
        temp += ", latPos:" + tt.substr(tt.rfind(':')+1);
        temp += ", latDis:" + std::to_string((double)x.lateralDistance);
//        temp += (boost::format(", relativePosition:{lonPos:%d, lonDis:%d, latPos:%d, latDis:%d}\n") % x.longitudinalPosition % x.longitudinalDistance % x.lateralPosition % x.lateralDistance).str();
//        std::cout << "    egoVehicleState: " << t.egoVehicleState << std::endl;
//        std::cout << "    otherVehicleState: " << t.otherVehicleState << std::endl;
        count++;
    }
    sprintf( sSituation, "situationSnapshot: {timeIndex:%ld, size:%ld}\n%s", situationSnapshot.timeIndex, situationSnapshot.situations.size(), temp.c_str());
    
//    std::cout << "start state log" << std::endl;
    // 查看状态中间结果
//    sState << "\033[31mrssStateSnapshot\033[0m: timeIndex: " << rssStateSnapshot.timeIndex;
//    sState << ", size: " << rssStateSnapshot.individualResponses.size() << std::endl;
    temp = "";
    for( auto &t : rssStateSnapshot.individualResponses ) {
        temp += "  - objectId:" + std::to_string(t.objectId);
        temp += ", situationId:" + std::to_string(t.situationId);
        auto& x = t.longitudinalState;
        temp += "\n    longitudinalState:{ ";
        temp += (x.isSafe?"safe":"\033[31mdanger\033[0m");
        tt = std::to_string(x.response);
        temp += ", response:" + tt.substr(tt.rfind(':')+1);
        temp += ", distance:" + std::to_string((double)x.rssStateInformation.currentDistance);
        temp += "(" + std::to_string((double)x.rssStateInformation.safeDistance) + ")}";
        auto& x2 = t.lateralStateRight;
        temp += "\n    lateralStateRight:{ ";
        temp += (x2.isSafe?"safe":"\033[31mdanger\033[0m");
        tt = std::to_string(x2.response);
        temp += ", response:" + tt.substr(tt.rfind(':')+1);
        temp += ", distance:" + std::to_string((double)x2.rssStateInformation.currentDistance);
        temp += "(" + std::to_string((double)x2.rssStateInformation.safeDistance) + ")}";
        auto& x3 = t.lateralStateLeft;
        temp += "\n    lateralStateLeft:{ ";
        temp += (x3.isSafe?"safe":"\033[31mdanger\033[0m");
        tt = std::to_string(x3.response);
        temp += ", response:" + tt.substr(tt.rfind(':')+1);
        temp += ", distance:" + std::to_string((double)x3.rssStateInformation.currentDistance);
        temp += "(" + std::to_string((double)x3.rssStateInformation.safeDistance) + ")}";
    }
    sprintf( sState, "rssStateSnapshot: {timeIndex:%ld, size:%ld}\n%s", rssStateSnapshot.timeIndex, rssStateSnapshot.individualResponses.size(), temp.c_str());
    
//    std::cout << "start response log" << std::endl;
    // 查看正确响应中间结果
    auto &t = properResponse;
    temp = "";
    tt = std::to_string(t.longitudinalResponse);
    temp += "  longitudinalResponse:" + tt.substr(tt.rfind(':')+1);
    tt = std::to_string(t.lateralResponseRight);
    temp += "\n  lateralResponseRight:" + tt.substr(tt.rfind(':')+1);
    tt = std::to_string(t.lateralResponseLeft);
    temp += "\n  lateralResponseLeft:" + tt.substr(tt.rfind(':')+1);
    sprintf( sResponse, "properResponse: {timeIndex:%ld, %s, dangerousObjects:%s}\n%s", properResponse.timeIndex, (t.isSafe?"safe":"\033[31mdanger\033[0m"), std::to_string(t.dangerousObjects).c_str(), temp.c_str());

//    std::cout << "start restriction log" << std::endl;
    // 查看安全限制
    auto &p = accelerationRestriction;
    temp = "";
    temp += "  longitudinalRange: " + std::to_string(p.longitudinalRange.minimum) + " ~ " + std::to_string(p.longitudinalRange.maximum);
    temp += "\n  lateralRightRange: " + std::to_string(p.lateralRightRange.minimum) + " ~ " + std::to_string(p.lateralRightRange.maximum);
    temp += "\n  lateralLeftRange: " + std::to_string(p.lateralLeftRange.minimum) + " ~ " + std::to_string(p.lateralLeftRange.maximum);
    sprintf( sRestriction, "accelerationRestriction: {timeIndex:%ld}\n%s", accelerationRestriction.timeIndex, temp.c_str());

//    std::cout << "end all log" << std::endl;
    return 1;
}

std::string ssWorld() {
    return std::string(sWorld);
}

std::string ssSituation() {
    return std::string(sSituation);
}

std::string ssState() {
    return std::string(sState);
}

std::string ssResponse() {
    return std::string(sResponse);
}

std::string ssRestriction() {
    return std::string(sRestriction);
}

std::string version() {
    return std::string("2020/5/15 10:59");
}

BOOST_PYTHON_MODULE(rssw) {
    class_<Lane>("Lane", init<double,double,double,double,double,int>())
        .def("str", &Lane::str)
//        .def("sum", &Lane::sum)
        .def_readwrite("x", &Lane::x)
        .def_readwrite("y", &Lane::y)
        .def_readwrite("length", &Lane::length)
        .def_readwrite("width", &Lane::width)
        .def_readwrite("heading", &Lane::heading)
        .def_readwrite("id", &Lane::id);
    class_<Vehicle>("Vehicle", init<double,double,double,double>())
        .def("str", &Vehicle::str)
        .def_readwrite("x", &Vehicle::x)
        .def_readwrite("y", &Vehicle::y)
        .def_readwrite("heading", &Vehicle::heading)
        .def_readwrite("velocity", &Vehicle::velocity);
    class_<VControl>("VControl", init<>())
        .def("str", &VControl::str)
        .def_readwrite("throttle", &VControl::throttle)
        .def_readwrite("brake", &VControl::brake)
        .def_readwrite("steer", &VControl::steer);
    def("RssCheck", RssCheck);
    def("ssWorld", ssWorld);
    def("ssSituation", ssSituation);
    def("ssState", ssState);
    def("ssResponse", ssResponse);
    def("ssRestriction", ssRestriction);
    def("version", version);
}
