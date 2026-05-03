#include "pfms_types.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>

#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"
#include "logger.h"

using namespace pfms::nav_msgs;

int main(int argc, char *argv[])
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0]
                  << " <ackerman_file> <skidsteer_file> [-advanced]" << std::endl;
        return 1;
    }

    std::string ackerman_filename  = argv[1];
    std::string skidsteer_filename = argv[2];
    pfms::MissionObjective objective = pfms::MissionObjective::BASIC;

    if (argc >= 4 && strcmp(argv[3], "-advanced") == 0) {
        objective = pfms::MissionObjective::ADVANCED;
        std::cout << "Advanced Mode Activated" << std::endl;
    }

    std::vector<pfms::geometry_msgs::Point> ackermanPoints;
    std::vector<pfms::geometry_msgs::Point> skidsteerPoints;

    if (!logger::loadPoints(ackerman_filename, ackermanPoints)) {
        std::cout << "Could not load points from file: " << ackerman_filename << std::endl;
        return 0;
    }
    if (!logger::loadPoints(skidsteer_filename, skidsteerPoints)) {
        std::cout << "Could not load points from file: " << skidsteer_filename << std::endl;
        return 0;
    }

    std::cout << "Size of Ackerman goals:"  << ackermanPoints.size()  << std::endl;
    std::cout << "Size of Skidsteer goals:" << skidsteerPoints.size() << std::endl;

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);
    controllers.back()->setTolerance(0.5);

    Mission mission(controllers);
    mission.setMissionObjective(objective);
    mission.setGoals(skidsteerPoints, pfms::SKIDSTEER);
    mission.setGoals(ackermanPoints,  pfms::ACKERMAN);

    mission.execute(true);

    auto start_time = std::chrono::system_clock::now();
    bool OK = false;

    while (!OK) {
        std::vector<unsigned int> progress = mission.status();
        if ((progress.front() == 100) && (progress.back() == 100)) {
            OK = true;
        } else {
            std::cout << "progress ..." << progress.front() << "% "
                      << progress.back() << "% " << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    for (auto controller : controllers) {
        delete controller;
    }
    return 0;
}
