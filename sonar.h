#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

/**
 * @brief Sonar sensor stub. Not used in this assignment.
 */
class Sonar : public Ranger
{
public:
    Sonar(pfms::PlatformType type);
    ~Sonar();

    std::vector<double> getData() override;
    double getAngularResolution(void) override;
    pfms::nav_msgs::Odometry getSensorPose(void) override;
    double getFieldOfView(void) override;
    double getMaxRange(void) override;
    double getMinRange(void) override;
    pfms::RangerType getSensingMethod(void) override;
};

#endif // SONAR_H
