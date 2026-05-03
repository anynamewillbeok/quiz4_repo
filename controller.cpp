#include "controller.h"
#include <chrono>
#include <iostream>

Controller::Controller()
    : platformType_(pfms::PlatformType::ACKERMAN),
      currentGoalIndex_(0),
      totalMissionDistance_(0.0),
      platformStatus_(pfms::PlatformStatus::IDLE),
      tolerance_(0.5),
      distanceToGoal_(0.0),
      timeToGoal_(0.0),
      distanceTravelled_(0.0),
      timeTravelled_(0.0),
      running_(false)
{
    odometry_ = {};
}

Controller::~Controller()
{
    running_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}

bool Controller::execute(bool start)
{
    if (start) {
        // Stop any existing thread first
        running_ = false;
        if (controlThread_.joinable()) {
            controlThread_.join();
        }
        running_ = true;
        controlThread_ = std::thread(&Controller::run, this);
    } else {
        running_ = false;
        // Don't join here - caller may be in a different context
    }
    return true;
}

pfms::PlatformStatus Controller::status(unsigned int& currentGoalIndex)
{
    {
        std::lock_guard<std::mutex> lock(goalsMutex_);
        currentGoalIndex = currentGoalIndex_;
    }
    std::lock_guard<std::mutex> lock(dataMutex_);
    return platformStatus_;
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals)
{
    std::lock_guard<std::mutex> lock(goalsMutex_);
    goals_ = goals;
    currentGoalIndex_ = 0;
    totalMissionDistance_ = 0.0;

    if (goals_.empty()) {
        return false;
    }

    // We need odometry to compute checkOriginToDestination - get current odo
    pfms::nav_msgs::Odometry currentOdo;
    {
        std::lock_guard<std::mutex> dlock(dataMutex_);
        currentOdo = odometry_;
    }

    bool allReachable = true;
    pfms::nav_msgs::Odometry origin = currentOdo;

    for (auto& goal : goals_) {
        double dist = 0.0, t = 0.0;
        pfms::nav_msgs::Odometry estimatedPose;
        bool reachable = checkOriginToDestination(origin, goal, dist, t, estimatedPose);
        if (!reachable || dist < 0.0) {
            allReachable = false;
            dist = 0.0;
        }
        totalMissionDistance_ += dist;
        origin = estimatedPose;
    }

    return allReachable;
}

pfms::PlatformType Controller::getPlatformType(void)
{
    return platformType_;
}

double Controller::distanceToGoal(void)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return distanceToGoal_;
}

double Controller::timeToGoal(void)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return timeToGoal_;
}

bool Controller::setTolerance(double tolerance)
{
    tolerance_ = tolerance;
    return true;
}

double Controller::distanceTravelled(void)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return distanceTravelled_;
}

double Controller::timeTravelled(void)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return timeTravelled_;
}

pfms::nav_msgs::Odometry Controller::getOdometry(void)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return odometry_;
}
