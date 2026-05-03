#include "laser.h"
#include <iostream>

Laser::Laser(pfms::PlatformType type)
    : Ranger(type)
{
    // Laser sensor parameters (Hokuyo-style, 270 deg FOV, 0.333 deg resolution)
    angularResolution_ = 0.3515625; // degrees per step (approx 1024 steps over 360 deg)
    fieldOfView_       = 0.0;       // POINT type: FOV reported as 0
    maxRange_          = 30.0;      // metres
    minRange_          = 0.1;       // metres
    sensingMethod_     = pfms::RangerType::POINT;

    // Static sensor pose offset in vehicle frame
    sensorPose_ = {};
    sensorPose_.position.z = 0.0;
    sensorPose_.yaw = 0.0;

    if (type == pfms::PlatformType::ACKERMAN) {
        // Audi: laser is 3.725m forward of the rear axle centre
        sensorPose_.position.x = 3.725;
        sensorPose_.position.y = 0.0;
    } else {
        // Husky: laser is co-located at the centre of the platform
        sensorPose_.position.x = 0.0;
        sensorPose_.position.y = 0.0;
    }
}

Laser::~Laser()
{
}

std::vector<double> Laser::getData()
{
    pfms::sensor_msgs::LaserScan scan;
    bool ok = pfmsConnectorPtr_->read(scan);
    if (!ok) {
        return {};
    }

    // Update angular resolution from actual scan if available
    if (scan.angle_increment > 0.0) {
        angularResolution_ = scan.angle_increment * 180.0 / M_PI;
    }
    if (scan.range_min > 0.0) minRange_ = scan.range_min;
    if (scan.range_max > 0.0) maxRange_ = scan.range_max;

    std::vector<double> result;
    result.reserve(scan.ranges.size());
    for (auto r : scan.ranges) {
        result.push_back(static_cast<double>(r));
    }
    return result;
}

double Laser::getAngularResolution(void)
{
    return angularResolution_;
}

pfms::nav_msgs::Odometry Laser::getSensorPose(void)
{
    return sensorPose_;
}

double Laser::getFieldOfView(void)
{
    return fieldOfView_;
}

double Laser::getMaxRange(void)
{
    return maxRange_;
}

double Laser::getMinRange(void)
{
    return minRange_;
}

pfms::RangerType Laser::getSensingMethod(void)
{
    return sensingMethod_;
}
