//#include "ad/rss/core/RssResponseResolving.hpp"
//#include "ad/rss/core/RssResponseTransformation.hpp"
//#include "ad/rss/core/RssSituationChecking.hpp"
//#include "ad/rss/core/RssSituationExtraction.hpp"
//#include "ad/rss/state/RssStateOperation.hpp"
#include "ad/rss/core/RssCheck.hpp"

void calculateOccupiedRegions(::ad::rss::world::OccupiedRegionVector &bounds, float x, float y) {
    ::ad::rss::world::OccupiedRegion r;
    r.segmentId = 43;
    r.lonRange.minimum=y-0.1;
    r.lonRange.maximum=y+0.1;
    r.latRange.minimum=x-0.1;
    r.latRange.maximum=x+0.1;
    bounds.push_back(r);
}

void calculateLatLonVelocities(::ad::rss::world::Velocity &v) {
    v.speedLonMin = ::ad::physics::Speed(20);
    v.speedLonMax = ::ad::physics::Speed(20);
    v.speedLatMin = ::ad::physics::Speed(0);
    v.speedLatMax = ::ad::physics::Speed(0);
}

int main() {
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
    calculateOccupiedRegions( otherVehicle.occupiedRegions, 0.5, 0.7 );
    calculateLatLonVelocities( otherVehicle.velocity );

    // 计算道路数据
    ::ad::rss::world::RoadArea roadArea;
    ::ad::rss::world::RoadSegment roadSegment;
    ::ad::rss::world::LaneSegment laneSegment;
    int length = 50;
    int width = 5;
    uint64_t lid;
    laneSegment.id = -4;
    if ( laneSegment.id < 0 )
        laneSegment.drivingDirection = ::ad::rss::world::LaneDrivingDirection::Negative;
    else
        laneSegment.drivingDirection = ::ad::rss::world::LaneDrivingDirection::Positive;
    laneSegment.length.minimum = length;
    laneSegment.length.maximum = length;
    laneSegment.width.minimum = width;
    laneSegment.width.maximum = width;
    lid = laneSegment.id;
    if ( lid < 0 ) lid += 0xffff;
    laneSegment.id = ((uint64_t)43 << 32) + ((uint64_t)1 << 16) + lid;
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
    
    return 1;
}