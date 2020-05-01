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
    calculateOccupiedRegions( egoVehicle.occupiedRegions, 0.1, 0.3 );
    calculateLatLonVelocities( egoVehicle.velocity );

    otherVehicle.objectId=24;
    otherVehicle.objectType=::ad::rss::world::ObjectType::OtherVehicle;
    calculateOccupiedRegions( otherVehicle.occupiedRegions, 0.5, 0.3 );
    calculateLatLonVelocities( otherVehicle.velocity );

    // 计算道路数据
    ::ad::rss::world::RoadArea roadArea;
    ::ad::rss::world::RoadSegment roadSegment;
    ::ad::rss::world::LaneSegment laneSegment;
    int length = 50;
    int width = 5;
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

    // 查看状况中间结果
    std::cout << "\033[31msituationSnapshot\033[0m: timeIndex: " << situationSnapshot.timeIndex;
    std::cout << ", size: " << situationSnapshot.situations.size() << std::endl;
    int count = 0;
    for( auto &t : situationSnapshot.situations ) {
        std::cout << "  -" << std::endl;
        std::cout << "    objectId: " << t.objectId << std::endl;
        std::cout << "    situationId: " << t.situationId << std::endl;
        std::cout << "    situationType: " << t.situationType << std::endl;
//        std::cout << "    egoVehicleState: " << t.egoVehicleState << std::endl;
//        std::cout << "    otherVehicleState: " << t.otherVehicleState << std::endl;
        std::cout << "    relativePosition: " << t.relativePosition << std::endl;
        count++;
    }
    
    // 查看状态中间结果
    std::cout << "\033[31mrssStateSnapshot\033[0m: timeIndex: " << rssStateSnapshot.timeIndex;
    std::cout << ", size: " << rssStateSnapshot.individualResponses.size() << std::endl;
    count = 0;
    for( auto &t : rssStateSnapshot.individualResponses ) {
        std::cout << "  -" << std::endl;
        std::cout << "    objectId: " << t.objectId << std::endl;
        std::cout << "    situationId: " << t.situationId << std::endl;
        std::cout << "    longitudinalState: " << t.longitudinalState << std::endl;
        std::cout << "    lateralStateRight: " << t.lateralStateRight << std::endl;
        std::cout << "    lateralStateLeft: " << t.lateralStateLeft << std::endl;
        count++;
    }
    
    // 查看正确响应中间结果
    std::cout << "\033[31mproperResponse\033[0m: timeIndex: " << properResponse.timeIndex << std::endl;
    auto &t = properResponse;
    std::cout << "  isSafe: " << t.isSafe << std::endl;
    std::cout << "  dangerousObjects: " << t.dangerousObjects << std::endl;
    std::cout << "  longitudinalResponse: " << t.longitudinalResponse << std::endl;
    std::cout << "  lateralResponseRight: " << t.lateralResponseRight << std::endl;
    std::cout << "  lateralResponseLeft: " << t.lateralResponseLeft << std::endl;

    // 查看安全限制
    std::cout << "\033[31maccelerationRestriction\033[0m: timeIndex: " << accelerationRestriction.timeIndex << std::endl;
    auto &p = accelerationRestriction;
    std::cout << "  longitudinalRange: " << p.longitudinalRange << std::endl;
    std::cout << "  lateralRightRange: " << p.lateralRightRange << std::endl;
    std::cout << "  lateralLeftRange: " << p.lateralLeftRange << std::endl;

    return 1;
}

BOOST_PYTHON_MODULE(rssw) {
    class_<Lane>("Lane", init<double,double,double,double,double>())
        .def_readwrite("x", &Lane::x)
        .def_readwrite("y", &Lane::y)
        .def_readwrite("length", &Lane::length)
        .def_readwrite("width", &Lane::width)
        .def_readwrite("heading", &Lane::heading);
    class_<Vehicle>("Vehicle", init<double,double,double,double>())
        .def_readwrite("x", &Vehicle::x)
        .def_readwrite("y", &Vehicle::y)
        .def_readwrite("heading", &Vehicle::heading)
        .def_readwrite("velocity", &Vehicle::velocity);
    def("RssCheck", RssCheck);
}
