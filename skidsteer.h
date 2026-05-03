#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

/**
 * @brief Controller for the Clearpath Husky skid-steer platform.
 *
 * Controls the platform by first rotating on the spot to face the goal,
 * then driving forward with minor heading corrections.
 * Max linear velocity: 2.0 m/s, max angular velocity: 1.0 rad/s.
 */
class Skidsteer : public Controller
{
public:
    /**
     * @brief Constructor. Sets platform type and creates PfmsConnector.
     */
    Skidsteer();

    /**
     * @brief Destructor. Stops control thread.
     */
    ~Skidsteer();

    /**
     * @brief Computes estimated distance and time from origin to goal.
     * Distance = Euclidean distance. Time includes rotation time plus travel time.
     * @param origin Starting odometry pose
     * @param goal Target point
     * @param distance Output: Euclidean distance in metres
     * @param time Output: estimated time (rotation + travel) in seconds
     * @param estimatedGoalPose Output: estimated pose at goal
     * @return true always (Husky can reach any point)
     */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;

protected:
    /**
     * @brief Skidsteer control loop. Runs in controlThread_.
     * Rotates to face goal, then drives forward with angular correction.
     * Sends SkidSteer commands, checks goal tolerance, accumulates distance/time.
     */
    void run() override;

private:
    unsigned long seq_; ///< Command sequence counter

    static constexpr double MAX_LINEAR_VEL  = 2.0; ///< Maximum forward speed [m/s]
    static constexpr double MAX_ANGULAR_VEL = 1.0; ///< Maximum turn rate [rad/s]

    /**
     * @brief Normalise an angle to the range [-pi, pi].
     * @param angle Input angle in radians
     * @return Normalised angle in radians
     */
    double normaliseAngle(double angle);
};

#endif // SKIDSTEER_H
