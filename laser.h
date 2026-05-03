#ifndef LASER_H
#define LASER_H

#include "ranger.h"

/**
 * @brief Laser range sensor implementation.
 *
 * POINT-type sensor. For Ackerman: placed 3.725m forward of centre.
 * For Husky: co-located at centre (0, 0).
 * Angular resolution: 0.333 deg (approx), FOV: 270 deg, range: 0.1-30m.
 */
class Laser : public Ranger
{
public:
    /**
     * @brief Constructor. Sets laser-specific parameters and sensor pose offset.
     * @param type Platform this laser is attached to
     */
    Laser(pfms::PlatformType type);

    /**
     * @brief Destructor.
     */
    ~Laser();

    /**
     * @brief Returns raw laser range data from the platform.
     * @return Vector of range values [m]
     */
    std::vector<double> getData() override;

    /**
     * @brief Returns angular resolution of the laser.
     * @return Angular resolution [deg]
     */
    double getAngularResolution(void) override;

    /**
     * @brief Returns the static sensor pose offset from vehicle centre.
     * @return Sensor pose (x, y, yaw) in vehicle frame stored as Odometry
     */
    pfms::nav_msgs::Odometry getSensorPose(void) override;

    /**
     * @brief Returns the field of view. Zero for POINT-type sensors.
     * @return Field of view [deg]
     */
    double getFieldOfView(void) override;

    /**
     * @brief Returns maximum valid range.
     * @return Maximum range [m]
     */
    double getMaxRange(void) override;

    /**
     * @brief Returns minimum valid range.
     * @return Minimum range [m]
     */
    double getMinRange(void) override;

    /**
     * @brief Returns the sensing method type.
     * @return RangerType::POINT
     */
    pfms::RangerType getSensingMethod(void) override;
};

#endif // LASER_H
