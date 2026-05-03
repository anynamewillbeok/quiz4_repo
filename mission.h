#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include "missioninterface.h"
#include "rangerinterface.h"
#include "ranger.h"
#include "pfmsconnector.h"

/**
 * @brief Mission class. Coordinates multiple platform controllers to execute
 * a series of goals, with optional obstacle detection in ADVANCED mode.
 *
 * BASIC mode:  Goals are passed to each platform and executed concurrently.
 * ADVANCED mode: After each goal, the laser sensor checks if the path to
 *               the next goal is obstructed. If so, that platform stops and
 *               the mission for it is abandoned.
 */
class Mission : public MissionInterface
{
public:
    /**
     * @brief Constructor for BASIC mode mission.
     * Sets MissionObjective to BASIC by default.
     * @param controllers Platform controllers to coordinate
     */
    Mission(std::vector<ControllerInterface*> controllers);

    /**
     * @brief Constructor for ADVANCED mode mission with ranger sensors.
     * @param controllers Platform controllers to coordinate
     * @param rangers Ranger sensors for obstacle detection (one per controller, matched by index)
     */
    Mission(std::vector<ControllerInterface*> controllers,
            std::vector<RangerInterface*> rangers);

    /**
     * @brief Destructor. Stops mission thread and all controllers.
     */
    ~Mission();

    /**
     * @brief Set goals for a specific platform type. Clears previous goals for that platform.
     * Passes the goals directly to the matching controller.
     * @param goals Ordered list of goal points
     * @param platform The platform type these goals are for
     */
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals,
                  pfms::PlatformType platform) override;

    /**
     * @brief Start or stop mission execution. Non-blocking call.
     * Calls execute() on all controllers and starts the mission coordination thread.
     * @param start true to start, false to stop all platforms immediately
     * @return true if mission can proceed
     */
    bool execute(bool start) override;

    /**
     * @brief Returns mission completion percentage for each platform.
     * Percentage = (distanceTravelled / totalMissionDistance) * 100, capped at 100.
     * @return Vector of completion percentages (0-100), one per controller
     */
    std::vector<unsigned int> status(void) override;

    /**
     * @brief Set the mission objective (BASIC or ADVANCED).
     * @param objective The desired mission objective
     * @return true always
     */
    bool setMissionObjective(pfms::MissionObjective objective) override;

    /**
     * @brief Returns total distance travelled by each platform since execution started.
     * @return Vector of distances [m], one per controller
     */
    std::vector<double> getDistanceTravelled() override;

    /**
     * @brief Returns total time each platform was moving since execution started.
     * @return Vector of times [s], one per controller
     */
    std::vector<double> getTimeMoving() override;

    /**
     * @brief Check if the mission was abandoned due to obstacle detection.
     * @return true if any platform's mission was abandoned
     */
    bool isAbandoned() const override;

    /**
     * @brief Get per-controller abandonment flags.
     * @return Vector of bools (true = that controller was stopped due to obstacle)
     */
    std::vector<bool> getAbandonedControllers() const override;

private:
    /**
     * @brief Mission coordination loop. Runs in missionThread_.
     * In ADVANCED mode: monitors controller goal progress, and after each goal
     * is reached, checks if the next goal is obstructed. If so, stops that controller.
     * In BASIC mode: simply waits for all controllers to reach IDLE.
     */
    void missionLoop();

    /**
     * @brief Check if the path from the platform's current position to nextGoal
     * is obstructed, using the corresponding ranger sensor.
     * @param controllerIdx Index into controllers_ and rangers_ vectors
     * @param nextGoal The goal point to check for obstruction
     * @return true if an obstacle is detected on the path
     */
    bool isPathObstructed(size_t controllerIdx,
                          pfms::geometry_msgs::Point nextGoal);

    std::vector<ControllerInterface*> controllers_; ///< Platform controllers
    std::vector<RangerInterface*> rangers_;         ///< Ranger sensors (one per controller)

    pfms::MissionObjective objective_; ///< BASIC or ADVANCED

    /// Goals per platform type, stored so we know total count for status()
    std::map<pfms::PlatformType, std::vector<pfms::geometry_msgs::Point>> goals_;

    /// Total estimated mission distance per controller (set at setGoals time)
    std::vector<double> totalDistances_;

    std::atomic<bool> abandoned_;           ///< True if any platform was abandoned
    std::vector<bool> abandonedControllers_; ///< Per-controller abandonment flag

    std::atomic<bool> running_;   ///< Signals missionThread_ to keep looping
    std::thread missionThread_;   ///< Runs missionLoop()

    mutable std::mutex missionMutex_; ///< Protects abandonedControllers_, goals_, totalDistances_
};

#endif // MISSION_H
