#include "skidsteer.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

Skidsteer::Skidsteer()
    : seq_(1)
{
    platformType_ = pfms::PlatformType::SKIDSTEER;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(platformType_);
}

Skidsteer::~Skidsteer()
{
    running_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}

double Skidsteer::normaliseAngle(double angle)
{
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool Skidsteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;
    distance = std::sqrt(dx * dx + dy * dy);

    double bearing = std::atan2(dy, dx);
    double angleError = std::abs(normaliseAngle(bearing - origin.yaw));

    double rotationTime = angleError / MAX_ANGULAR_VEL;
    double travelTime   = distance / MAX_LINEAR_VEL;
    time = rotationTime + travelTime;

    estimatedGoalPose = origin;
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = bearing;

    return true;
}

void Skidsteer::run()
{
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        platformStatus_ = pfms::PlatformStatus::RUNNING;
    }

    auto lastTime = std::chrono::steady_clock::now();
    pfms::nav_msgs::Odometry prevOdo = {};
    bool hasPrevOdo = false;

    while (running_) {
        pfms::geometry_msgs::Point currentGoal;
        unsigned int goalIdx;
        {
            std::lock_guard<std::mutex> lock(goalsMutex_);
            goalIdx = currentGoalIndex_;
            if (goalIdx >= goals_.size()) {
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

        // Time tracking
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;

        if (hasPrevOdo) {
            double dx = odo.position.x - prevOdo.position.x;
            double dy = odo.position.y - prevOdo.position.y;
            double dDist = std::sqrt(dx * dx + dy * dy);
            double speed = std::sqrt(odo.linear.x * odo.linear.x +
                                     odo.linear.y * odo.linear.y);
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

        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            odometry_ = odo;
        }

        // Distance to goal
        double dx = currentGoal.x - odo.position.x;
        double dy = currentGoal.y - odo.position.y;
        double distToGoal = std::sqrt(dx * dx + dy * dy);

        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            distanceToGoal_ = distToGoal;
            timeToGoal_ = distToGoal / MAX_LINEAR_VEL;
        }

        if (distToGoal <= tolerance_) {
            // Stop
            pfms::commands::SkidSteer stopCmd{seq_++, 0.0, 0.0};
            pfmsConnectorPtr_->send(stopCmd);

            {
                std::lock_guard<std::mutex> lock(goalsMutex_);
                currentGoalIndex_++;
                if (currentGoalIndex_ >= goals_.size()) {
                    break;
                }
            }
            continue;
        }

        // Compute bearing to goal
        double bearing = std::atan2(dy, dx);
        double angleError = normaliseAngle(bearing - odo.yaw);

        pfms::commands::SkidSteer cmd;
        cmd.seq = seq_++;

        const double ANGLE_THRESHOLD = 0.15; // rad - switch between rotate and drive modes

        if (std::abs(angleError) > ANGLE_THRESHOLD) {
            // Rotate on spot to face goal
            double angVel = (angleError > 0) ? MAX_ANGULAR_VEL : -MAX_ANGULAR_VEL;
            // Proportional slow-down near target angle
            if (std::abs(angleError) < 0.5) {
                angVel *= (std::abs(angleError) / 0.5);
                if (std::abs(angVel) < 0.2) angVel = (angVel > 0) ? 0.2 : -0.2;
            }
            cmd.turn_l_r = angVel;
            cmd.move_f_b = 0.0;
        } else {
            // Drive toward goal with proportional angular correction
            double linVel = MAX_LINEAR_VEL;
            // Slow down when approaching goal
            if (distToGoal < 2.0) {
                linVel = std::max(0.3, distToGoal * MAX_LINEAR_VEL / 2.0);
            }
            double angCorr = angleError * 1.5;
            if (angCorr > MAX_ANGULAR_VEL)  angCorr = MAX_ANGULAR_VEL;
            if (angCorr < -MAX_ANGULAR_VEL) angCorr = -MAX_ANGULAR_VEL;
            cmd.turn_l_r = angCorr;
            cmd.move_f_b = linVel;
        }

        pfmsConnectorPtr_->send(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Full stop
    pfms::commands::SkidSteer stopCmd{seq_++, 0.0, 0.0};
    pfmsConnectorPtr_->send(stopCmd);

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        platformStatus_ = pfms::PlatformStatus::IDLE;
        distanceToGoal_ = 0.0;
        timeToGoal_ = 0.0;
    }
    running_ = false;
}
