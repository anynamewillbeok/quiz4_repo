#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

/**
 * @brief Controller for the Audi R8 Ackerman-steered platform.
 *
 * Uses the Audi library to compute steering angles and arc distances.
 * Inherits threading, goal management, and odometry tracking from Controller.
 */
class Ackerman : public Controller
{
public:
    /**
     * @brief Constructor. Sets platform type and initialises Audi library instance.
     */
    Ackerman();

    /**
     * @brief Destructor. Stops control thread.
     */
    ~Ackerman();

    /**
     * @brief Checks reachability and computes distance/time from origin to goal.
     * Delegates to Audi::checkOriginToDestination().
     * @param origin Starting odometry pose
     * @param goal Target point
     * @param distance Output: arc distance in metres (-1 if unreachable)
     * @param time Output: estimated travel time in seconds (-1 if unreachable)
     * @param estimatedGoalPose Output: estimated pose when goal is reached
     * @return true if goal is reachable via constant steering
     */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;

protected:
    /**
     * @brief Ackerman control loop. Runs in controlThread_.
     * Reads odometry, computes steering via Audi, sends commands,
     * checks goal tolerance, accumulates distance and time.
     */
    void run() override;

private:
    Audi audi_;       ///< Audi library instance for kinematic computations
    unsigned long seq_; ///< Command sequence counter (must be strictly increasing)
};

#endif // ACKERMAN_H
