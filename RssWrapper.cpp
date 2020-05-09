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

void calculateOccupiedRegions(::ad::rss::world::OccupiedRegionVector &bounds, float x, float y) {
    ::ad::rss::world::OccupiedRegion r;
    r.segmentId = 43;
    r.lonRange.minimum=::ad::physics::ParametricValue(y);
    r.lonRange.maximum=::ad::physics::ParametricValue(y);
    r.latRange.minimum=::ad::physics::ParametricValue(x);
    r.latRange.maximum=::ad::physics::ParametricValue(x);
    bounds.push_back(r);
}

void calculateLatLonVelocities(::ad::rss::world::Velocity &v) {
    v.speedLonMin = ::ad::physics::Speed(20);
    v.speedLonMax = ::ad::physics::Speed(20);
    v.speedLatMin = ::ad::physics::Speed(0);
    v.speedLatMax = ::ad::physics::Speed(0);
}

int RssCheck(Lane lane, Vehicle ego, Vehicle other) {
    ::ad::rss::core::RssCheck rssCheck;
    ::ad::rss::situation::SituationSnapshot situationSnapshot;
    ::ad::rss::state::RssStateSnapshot rssStateSnapshot;
    ::ad::rss::state::ProperResponse properResponse;
    ::ad::rss::world::WorldModel worldModel;
    ::ad::rss::world::AccelerationRestriction accelerationRestriction;
    
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
    calculateOccupiedRegions( egoVehicle.occupiedRegions, ego.x, ego.y );
    calculateLatLonVelocities( egoVehicle.velocity );

    otherVehicle.objectId=24;
    otherVehicle.objectType=::ad::rss::world::ObjectType::OtherVehicle;
    calculateOccupiedRegions( otherVehicle.occupiedRegions, other.x, other.y );
    calculateLatLonVelocities( otherVehicle.velocity );

    // 计算道路数据
    ::ad::rss::world::RoadArea roadArea;
    ::ad::rss::world::RoadSegment roadSegment;
    ::ad::rss::world::LaneSegment laneSegment;
    int length = lane.length;
    int width = lane.width;
    uint64_t lid;
    laneSegment.id = 43;
    if ( laneSegment.id < 0 )
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
    
    // 调用RSS检查主函数
    rssCheck.calculateAccelerationRestriction(worldModel, situationSnapshot, rssStateSnapshot, properResponse, accelerationRestriction);

    // 将安全限制转化为控制参数，todo

//    sWorld.clear();
//    sSituation.clear();
//    sState.clear();
//    sResponse.clear();
//    sRestriction.clear();
    
//    sWorld << worldModel << std::endl;
//    std::cout << worldModel << std::endl;

    // 查看输入适配
    auto& dyn = worldModel.defaultEgoVehicleRssDynamics;
    std::string temp, tt;
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
        v = x.object;
        temp += "\n    object: {objectId:" + std::to_string(v.objectId);
        tt = std::to_string(v.objectType);
        temp += ", objectType:" + tt.substr(tt.rfind(':')+1);
        s = v.velocity;
        temp += ", velocity: {lon: " + std::to_string(s.speedLonMin) + "~" + std::to_string(s.speedLonMax);
        temp += ", lat: " + std::to_string(s.speedLatMin) + "~" + std::to_string(s.speedLatMax) + "}}";
        for( auto& r : v.occupiedRegions) {
            temp += "\n      - occupiedRegion: {segmentId:" + std::to_string(r.segmentId);
            temp += ", lonRange: " + std::to_string(r.lonRange.minimum) + " ~ " + std::to_string(r.lonRange.maximum);
            temp += ", latRange: " + std::to_string(r.latRange.minimum) + " ~ " + std::to_string(r.latRange.maximum) + "}";
        }
//        temp += "\n    objectRssDynamics" + std::to_string(x.objectRssDynamics);
    }
    sprintf( sWorld, "world: {timeIndex:%ld, size:%ld}%s", worldModel.timeIndex, worldModel.scenes.size(), temp.c_str());
    
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

    // 查看安全限制
    auto &p = accelerationRestriction;
    temp = "";
    temp += "  longitudinalRange: " + std::to_string(p.longitudinalRange.minimum) + " ~ " + std::to_string(p.longitudinalRange.maximum);
    temp += "\n  lateralRightRange: " + std::to_string(p.lateralRightRange.minimum) + " ~ " + std::to_string(p.lateralRightRange.maximum);
    temp += "\n  lateralLeftRange: " + std::to_string(p.lateralLeftRange.minimum) + " ~ " + std::to_string(p.lateralLeftRange.maximum);
    sprintf( sRestriction, "accelerationRestriction: {timeIndex:%ld}\n%s", accelerationRestriction.timeIndex, temp.c_str());

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

BOOST_PYTHON_MODULE(rssw) {
    class_<Lane>("Lane", init<double,double,double,double,double>())
        .def("str", &Lane::str)
//        .def("sum", &Lane::sum)
        .def_readwrite("x", &Lane::x)
        .def_readwrite("y", &Lane::y)
        .def_readwrite("length", &Lane::length)
        .def_readwrite("width", &Lane::width)
        .def_readwrite("heading", &Lane::heading);
    class_<Vehicle>("Vehicle", init<double,double,double,double>())
        .def("str", &Vehicle::str)
        .def_readwrite("x", &Vehicle::x)
        .def_readwrite("y", &Vehicle::y)
        .def_readwrite("heading", &Vehicle::heading)
        .def_readwrite("velocity", &Vehicle::velocity);
    def("RssCheck", RssCheck);
    def("ssWorld", ssWorld);
    def("ssSituation", ssSituation);
    def("ssState", ssState);
    def("ssResponse", ssResponse);
    def("ssRestriction", ssRestriction);
}
