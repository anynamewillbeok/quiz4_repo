#include "sonar.h"

Sonar::Sonar(pfms::PlatformType type)
    : Ranger(type)
{
    angularResolution_ = 0.0;
    fieldOfView_       = 30.0; // CONE type
    maxRange_          = 5.0;
    minRange_          = 0.2;
    sensingMethod_     = pfms::RangerType::CONE;
    sensorPose_ = {};

    if (type == pfms::PlatformType::ACKERMAN) {
        sensorPose_.position.x = 3.725;
        sensorPose_.position.y = 0.0;
        sensorPose_.position.z = 0.2; // 0.2m above laser
    } else {
        sensorPose_.position.x = 0.0;
        sensorPose_.position.y = 0.0;
        sensorPose_.position.z = 0.2;
    }
}

Sonar::~Sonar() {}

std::vector<double> Sonar::getData()
{
    pfms::sensor_msgs::Sonar sonar;
    bool ok = pfmsConnectorPtr_->read(sonar);
    if (!ok) return {};
    if (sonar.range < sonar.range_min || sonar.range > sonar.range_max) return {};
    return {sonar.range};
}

double Sonar::getAngularResolution(void) { return angularResolution_; }
pfms::nav_msgs::Odometry Sonar::getSensorPose(void) { return sensorPose_; }
double Sonar::getFieldOfView(void) { return fieldOfView_; }
double Sonar::getMaxRange(void) { return maxRange_; }
double Sonar::getMinRange(void) { return minRange_; }
pfms::RangerType Sonar::getSensingMethod(void) { return sensingMethod_; }
