#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include "pfms_types.h"
#include "pfmsconnector.h"
#include <cmath>

/**
 * @brief Base class for all ranger (sensor) types.
 *
 * Provides shared PfmsConnector for sensor data retrieval and stores the
 * static sensor pose offset from the vehicle centre. Derived classes set
 * sensor-specific parameters (FOV, range, angular resolution, type).
 */
class Ranger : public RangerInterface
{
public:
    /**
     * @brief Constructor. Creates PfmsConnector for the given platform type.
     * @param type The platform this ranger is attached to
     */
    Ranger(pfms::PlatformType type);

    /**
     * @brief Checks if the path to the next goal is obstructed.
     *
     * Reads the latest sensor data and checks whether any range return
     * lies in the angular window pointing toward nextGoal, at a range
     * less than the distance to that goal. Uses the platform odometry
     * and static sensor offset to compute the world-frame bearing.
     *
     * @param platformOdo Current odometry of the platform (world frame)
     * @param nextGoal The next goal to check for obstruction
     * @return true if an obstacle is detected on the path to nextGoal
     */
    bool isObstructed(pfms::nav_msgs::Odometry platformOdo,
                      pfms::geometry_msgs::Point nextGoal);

protected:
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; ///< Platform communication handle
    pfms::PlatformType type_;                          ///< Platform type this ranger is on
    pfms::nav_msgs::Odometry sensorPose_;              ///< Static offset of sensor from vehicle centre

    // Sensor parameters set by derived class constructors
    double angularResolution_; ///< Angular resolution [deg]
    double fieldOfView_;       ///< Field of view [deg] (0 for POINT type)
    double maxRange_;          ///< Maximum valid range [m]
    double minRange_;          ///< Minimum valid range [m]
    pfms::RangerType sensingMethod_; ///< POINT or CONE
};

#endif // RANGER_H
