#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

/**
 * @brief Base Controller class providing common functionality for all platform controllers.
 *
 * Implements shared threading, odometry tracking, distance/time accumulation,
 * and goal management. Derived classes implement platform-specific control loops
 * via the pure virtual run() method.
 */
class Controller : public ControllerInterface
{
public:
    /**
     * @brief Default constructor. Initialises all shared state to safe defaults.
     */
    Controller();

    /**
     * @brief Destructor. Stops the control thread safely before destruction.
     */
    virtual ~Controller();

    /**
     * @brief Start or stop controller execution. Non-blocking.
     * @param start true to begin executing goals, false to stop immediately
     * @return true if command accepted
     */
    bool execute(bool start) override;

    /**
     * @brief Returns platform status and current goal index.
     * @param currentGoalIndex Updated with the index of the goal currently being pursued
     * @return Current PlatformStatus (IDLE, RUNNING, STOPPING)
     */
    pfms::PlatformStatus status(unsigned int& currentGoalIndex) override;

    /**
     * @brief Set goals to pursue in order. Clears any previous goals.
     * @param goals Ordered vector of goal points
     * @return true if all goals are reachable in sequence
     */
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;

    /**
     * @brief Returns the platform type for this controller.
     * @return PlatformType enum value
     */
    pfms::PlatformType getPlatformType(void) override;

    /**
     * @brief Returns estimated distance to the current goal.
     * @return Distance in metres
     */
    double distanceToGoal(void) override;

    /**
     * @brief Returns estimated time to reach the current goal.
     * @return Time in seconds
     */
    double timeToGoal(void) override;

    /**
     * @brief Set the tolerance radius for considering a goal reached.
     * @param tolerance Distance in metres
     * @return true always
     */
    bool setTolerance(double tolerance) override;

    /**
     * @brief Returns total distance driven since execution started.
     * @return Distance in metres
     */
    double distanceTravelled(void) override;

    /**
     * @brief Returns total time the platform was in motion.
     * @return Time in seconds
     */
    double timeTravelled(void) override;

    /**
     * @brief Returns the most recently read odometry.
     * @return Odometry struct with position and velocity
     */
    pfms::nav_msgs::Odometry getOdometry(void) override;

protected:
    /**
     * @brief Pure virtual platform-specific control loop. Runs in controlThread_.
     * Derived classes implement steering/velocity logic, advancing goals, and
     * updating all tracked state variables.
     */
    virtual void run() = 0;

    // --- Communication ---
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; ///< Handles ROS communication

    // --- Platform identity ---
    pfms::PlatformType platformType_; ///< Set by derived class constructor

    // --- Goals ---
    std::vector<pfms::geometry_msgs::Point> goals_; ///< Ordered list of goals to pursue
    unsigned int currentGoalIndex_;                 ///< Index of goal currently being pursued
    double totalMissionDistance_;                   ///< Sum of estimated distances for all goal legs

    // --- Controller state ---
    pfms::PlatformStatus platformStatus_; ///< Current IDLE/RUNNING/STOPPING status
    double tolerance_;                    ///< Goal-reached radius in metres
    double distanceToGoal_;               ///< Estimated distance to current goal
    double timeToGoal_;                   ///< Estimated time to current goal

    // --- Accumulated tracking ---
    double distanceTravelled_; ///< Total distance driven since execute(true)
    double timeTravelled_;     ///< Total time spent moving since execute(true)

    // --- Odometry ---
    pfms::nav_msgs::Odometry odometry_; ///< Latest odometry reading

    // --- Threading ---
    std::atomic<bool> running_;    ///< Signals control thread to keep looping
    std::thread controlThread_;    ///< Runs the platform-specific control loop

    // --- Mutexes ---
    std::mutex goalsMutex_; ///< Protects goals_, currentGoalIndex_, totalMissionDistance_
    std::mutex dataMutex_;  ///< Protects odometry_, distanceTravelled_, timeTravelled_,
                            ///<   distanceToGoal_, timeToGoal_, platformStatus_
};

#endif // CONTROLLER_H
