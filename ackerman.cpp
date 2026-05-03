#include "ackerman.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

Ackerman::Ackerman()
    : seq_(1)
{
    platformType_ = pfms::PlatformType::ACKERMAN;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(platformType_);
}

Ackerman::~Ackerman()
{
    running_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    return audi_.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);
}

void Ackerman::run()
{
    // Set status to RUNNING
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        platformStatus_ = pfms::PlatformStatus::RUNNING;
    }

    auto lastTime = std::chrono::steady_clock::now();
    pfms::nav_msgs::Odometry prevOdo = {};
    bool hasPrevOdo = false;

    while (running_) {
        // Get current goal
        pfms::geometry_msgs::Point currentGoal;
        unsigned int goalIdx;
        {
            std::lock_guard<std::mutex> lock(goalsMutex_);
            goalIdx = currentGoalIndex_;
            if (goalIdx >= goals_.size()) {
                // All goals reached
                break;
            }
            currentGoal = goals_[goalIdx];
        }

        // Read odometry
        pfms::nav_msgs::Odometry odo;
        bool odoOK = pfmsConnectorPtr_->read(odo);
        if (!odoOK) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Compute time delta
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;

        // Accumulate distance and time if moving
        if (hasPrevOdo) {
            double dx = odo.position.x - prevOdo.position.x;
            double dy = odo.position.y - prevOdo.position.y;
            double dDist = std::sqrt(dx * dx + dy * dy);
            double speed = std::sqrt(odo.linear.x * odo.linear.x + odo.linear.y * odo.linear.y);
            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                distanceTravelled_ += dDist;
                if (speed > 0.05) {
                    timeTravelled_ += dt;
                }
            }
        }
        prevOdo = odo;
        hasPrevOdo = true;

        // Update odometry in shared state
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            odometry_ = odo;
        }

        // Check if goal reached
        double dx = currentGoal.x - odo.position.x;
        double dy = currentGoal.y - odo.position.y;
        double distToGoal = std::sqrt(dx * dx + dy * dy);

        if (distToGoal <= tolerance_) {
            // Goal reached - advance to next
            std::lock_guard<std::mutex> lock(goalsMutex_);
            currentGoalIndex_++;
            if (currentGoalIndex_ >= goals_.size()) {
                break;
            }
            continue;
        }

        // Compute steering to goal
        double steering = 0.0;
        double dist = 0.0;
        bool reachable = audi_.computeSteering(odo, currentGoal, steering, dist);

        // Update distanceToGoal and timeToGoal
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            distanceToGoal_ = distToGoal;
            // Recalibrate totalMissionDistance_ using actual dist to goal
            timeToGoal_ = dist > 0.0 ? dist / 3.6 : 0.0; // rough: 3.6m/s steady state
        }

        // Send drive command
        pfms::commands::Ackerman cmd;
        cmd.seq = seq_++;
        cmd.brake = 0.0;
        cmd.throttle = reachable ? 0.2 : 0.0;
        cmd.steering = reachable ? steering : 0.0;

        pfmsConnectorPtr_->send(cmd);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop the vehicle
    pfms::commands::Ackerman stopCmd;
    stopCmd.seq = seq_++;
    stopCmd.brake = 8000.0;
    stopCmd.throttle = 0.0;
    stopCmd.steering = 0.0;
    pfmsConnectorPtr_->send(stopCmd);

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        platformStatus_ = pfms::PlatformStatus::IDLE;
        distanceToGoal_ = 0.0;
        timeToGoal_ = 0.0;
    }
    running_ = false;
}
