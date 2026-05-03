#include "ranger.h"
#include <cmath>
#include <iostream>

namespace {
/**
 * @brief Normalise an angle to [-pi, pi].
 */
double normaliseAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
} // anonymous namespace

Ranger::Ranger(pfms::PlatformType type)
    : type_(type),
      angularResolution_(0.0),
      fieldOfView_(0.0),
      maxRange_(0.0),
      minRange_(0.0),
      sensingMethod_(pfms::RangerType::POINT)
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(type);
    sensorPose_ = {};
}

bool Ranger::isObstructed(pfms::nav_msgs::Odometry platformOdo,
                           pfms::geometry_msgs::Point nextGoal)
{
    // Get raw range data from the sensor
    std::vector<double> ranges = getData();
    if (ranges.empty()) {
        return false;
    }

    // Sensor is offset from vehicle centre by sensorPose_ (static, vehicle frame)
    // The sensor position in world frame:
    double sensorWorldX = platformOdo.position.x
                          + sensorPose_.position.x * std::cos(platformOdo.yaw)
                          - sensorPose_.position.y * std::sin(platformOdo.yaw);
    double sensorWorldY = platformOdo.position.y
                          + sensorPose_.position.x * std::sin(platformOdo.yaw)
                          + sensorPose_.position.y * std::cos(platformOdo.yaw);

    // Bearing from sensor to next goal in world frame
    double dx = nextGoal.x - sensorWorldX;
    double dy = nextGoal.y - sensorWorldY;
    double distToGoal = std::sqrt(dx * dx + dy * dy);
    double bearingToGoal = std::atan2(dy, dx);

    // Convert to sensor frame: subtract platform yaw and sensor yaw offset
    double sensorYaw = platformOdo.yaw + sensorPose_.yaw;
    double bearingInSensorFrame = normaliseAngle(bearingToGoal - sensorYaw);

    // Build angular window: ±(angularResolution_ * 3) around bearing to goal
    // Use at least 3 beam widths to account for sensor noise and box width
    double halfWindow = std::max(std::abs(bearingInSensorFrame),
                                 3.0 * angularResolution_ * M_PI / 180.0);
    halfWindow = std::max(halfWindow, 10.0 * M_PI / 180.0); // minimum 10 deg window

    // LaserScan: angle_min is typically -pi/2 or similar, iterate over all beams
    // We use angularResolution_ to compute each beam's angle in sensor frame
    double angleMin = -(static_cast<double>(ranges.size() - 1) / 2.0)
                       * angularResolution_ * M_PI / 180.0;

    for (size_t i = 0; i < ranges.size(); ++i) {
        double r = ranges[i];
        if (r < minRange_ || r > maxRange_) continue; // discard invalid

        double beamAngle = angleMin + i * angularResolution_ * M_PI / 180.0;
        double angleDiff = std::abs(normaliseAngle(beamAngle - bearingInSensorFrame));

        if (angleDiff <= halfWindow) {
            // This beam points roughly toward the goal
            if (r < distToGoal) {
                // Something is closer than the goal - obstruction detected
                return true;
            }
        }
    }
    return false;
}
