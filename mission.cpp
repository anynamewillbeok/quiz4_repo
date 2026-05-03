#include "mission.h"
#include "ranger.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>

Mission::Mission(std::vector<ControllerInterface*> controllers)
    : controllers_(controllers),
      objective_(pfms::MissionObjective::BASIC),
      abandoned_(false),
      running_(false)
{
    abandonedControllers_.resize(controllers_.size(), false);
    totalDistances_.resize(controllers_.size(), 0.0);
}

Mission::Mission(std::vector<ControllerInterface*> controllers,
                 std::vector<RangerInterface*> rangers)
    : controllers_(controllers),
      rangers_(rangers),
      objective_(pfms::MissionObjective::ADVANCED),
      abandoned_(false),
      running_(false)
{
    abandonedControllers_.resize(controllers_.size(), false);
    totalDistances_.resize(controllers_.size(), 0.0);
}

Mission::~Mission()
{
    running_ = false;
    if (missionThread_.joinable()) {
        missionThread_.join();
    }
}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals,
                       pfms::PlatformType platform)
{
    std::lock_guard<std::mutex> lock(missionMutex_);
    goals_[platform] = goals;

    // Find the controller for this platform and pass goals to it
    for (size_t i = 0; i < controllers_.size(); ++i) {
        if (controllers_[i]->getPlatformType() == platform) {
            controllers_[i]->setGoals(goals);

            // Estimate total mission distance from checkOriginToDestination chain
            double total = 0.0;
            pfms::nav_msgs::Odometry origin = controllers_[i]->getOdometry();
            for (auto& goal : goals) {
                double dist = 0.0, t = 0.0;
                pfms::nav_msgs::Odometry est;
                bool ok = controllers_[i]->checkOriginToDestination(origin, goal, dist, t, est);
                if (ok && dist >= 0.0) {
                    total += dist;
                }
                origin = est;
            }
            totalDistances_[i] = total;
            break;
        }
    }
}

bool Mission::execute(bool start)
{
    if (start) {
        // Reset abandonment state
        abandoned_ = false;
        {
            std::lock_guard<std::mutex> lock(missionMutex_);
            std::fill(abandonedControllers_.begin(), abandonedControllers_.end(), false);
        }

        // Start all controllers
        for (auto& c : controllers_) {
            c->execute(true);
        }

        // Stop any existing mission thread
        running_ = false;
        if (missionThread_.joinable()) {
            missionThread_.join();
        }

        // Start mission coordination thread
        running_ = true;
        missionThread_ = std::thread(&Mission::missionLoop, this);
    } else {
        running_ = false;
        for (auto& c : controllers_) {
            c->execute(false);
        }
    }
    return true;
}

std::vector<unsigned int> Mission::status(void)
{
    std::vector<unsigned int> result;
    result.reserve(controllers_.size());

    for (size_t i = 0; i < controllers_.size(); ++i) {
        double travelled = controllers_[i]->distanceTravelled();
        double total = 0.0;
        {
            std::lock_guard<std::mutex> lock(missionMutex_);
            total = totalDistances_[i];
        }

        unsigned int pct = 0;
        if (total > 0.0) {
            pct = static_cast<unsigned int>(std::min(100.0, (travelled / total) * 100.0));
        }

        // If controller is IDLE and we have goals, it's done — return 100
        unsigned int goalIdx = 0;
        pfms::PlatformStatus st = controllers_[i]->status(goalIdx);
        if (st == pfms::PlatformStatus::IDLE && total > 0.0) {
            pct = 100;
        }

        result.push_back(pct);
    }
    return result;
}

bool Mission::setMissionObjective(pfms::MissionObjective objective)
{
    objective_ = objective;
    return true;
}

std::vector<double> Mission::getDistanceTravelled()
{
    std::vector<double> result;
    result.reserve(controllers_.size());
    for (auto& c : controllers_) {
        result.push_back(c->distanceTravelled());
    }
    return result;
}

std::vector<double> Mission::getTimeMoving()
{
    std::vector<double> result;
    result.reserve(controllers_.size());
    for (auto& c : controllers_) {
        result.push_back(c->timeTravelled());
    }
    return result;
}

bool Mission::isAbandoned() const
{
    return abandoned_.load();
}

std::vector<bool> Mission::getAbandonedControllers() const
{
    std::lock_guard<std::mutex> lock(missionMutex_);
    return abandonedControllers_;
}

bool Mission::isPathObstructed(size_t controllerIdx,
                                pfms::geometry_msgs::Point nextGoal)
{
    if (controllerIdx >= rangers_.size() || rangers_[controllerIdx] == nullptr) {
        return false;
    }

    // Downcast to Ranger to access isObstructed()
    Ranger* ranger = dynamic_cast<Ranger*>(rangers_[controllerIdx]);
    if (ranger == nullptr) {
        return false;
    }

    pfms::nav_msgs::Odometry odo = controllers_[controllerIdx]->getOdometry();
    return ranger->isObstructed(odo, nextGoal);
}

void Mission::missionLoop()
{
    if (objective_ == pfms::MissionObjective::BASIC) {
        // BASIC: just wait for all controllers to finish
        while (running_) {
            bool allDone = true;
            for (auto& c : controllers_) {
                unsigned int idx = 0;
                pfms::PlatformStatus st = c->status(idx);
                if (st != pfms::PlatformStatus::IDLE) {
                    allDone = false;
                    break;
                }
            }
            if (allDone) {
                running_ = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return;
    }

    // ADVANCED mode: monitor each controller's goal index
    // When a goal is reached, check if next goal is obstructed
    std::vector<unsigned int> lastGoalIndex(controllers_.size(), 0);
    std::vector<bool> controllerDone(controllers_.size(), false);

    // Retrieve goals per controller to know when to check next goal
    std::vector<std::vector<pfms::geometry_msgs::Point>> controllerGoals(controllers_.size());
    {
        std::lock_guard<std::mutex> lock(missionMutex_);
        for (size_t i = 0; i < controllers_.size(); ++i) {
            pfms::PlatformType pt = controllers_[i]->getPlatformType();
            if (goals_.count(pt)) {
                controllerGoals[i] = goals_.at(pt);
            }
        }
    }

    while (running_) {
        bool allDone = true;

        for (size_t i = 0; i < controllers_.size(); ++i) {
            if (controllerDone[i]) continue;

            allDone = false;

            unsigned int currentGoalIdx = 0;
            pfms::PlatformStatus st = controllers_[i]->status(currentGoalIdx);

            if (st == pfms::PlatformStatus::IDLE) {
                // Controller finished all goals without abandonment
                controllerDone[i] = true;
                continue;
            }

            // Detect when the controller advances to a new goal
            if (currentGoalIdx > lastGoalIndex[i]) {
                lastGoalIndex[i] = currentGoalIdx;

                // Check if there is a next goal to evaluate
                size_t nextGoalIdx = currentGoalIdx;
                if (nextGoalIdx < controllerGoals[i].size()) {
                    pfms::geometry_msgs::Point nextGoal = controllerGoals[i][nextGoalIdx];

                    if (isPathObstructed(i, nextGoal)) {
                        // Stop this controller
                        controllers_[i]->execute(false);
                        {
                            std::lock_guard<std::mutex> lock(missionMutex_);
                            abandonedControllers_[i] = true;
                        }
                        abandoned_ = true;
                        controllerDone[i] = true;
                        std::cout << "[Mission] Controller " << i
                                  << " abandoned: obstacle at next goal." << std::endl;
                    }
                }
            }
        }

        if (allDone) {
            running_ = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
