#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

using namespace mbot_lcm_msgs;

pose2D_t nextPose;

// Function to move straight by `distance` meters
pose2D_t goStraight(float distance) {
    // pose2D_t nextPose;
    nextPose.x = nextPose.x + distance;
    nextPose.y = nextPose.y;
    nextPose.theta = 0.0;
    return nextPose;
}

// Function to turn left
pose2D_t turnLeft(float turn_distance) {
    nextPose.x = nextPose.x;
    nextPose.y = nextPose.y + turn_distance;
    nextPose.theta = 0.0;
    return nextPose;
}

// Function to turn right
pose2D_t turnRight(float turn_distance) {
    nextPose.x = nextPose.x;
    nextPose.y = nextPose.y - turn_distance;
    nextPose.theta = 0.0;
    return nextPose;
}

// Function to generate the square path
path2D_t generatePath(int numTimes) {
    path2D_t path;
    path.path.resize(numTimes*1);

    path.path.push_back(goStraight(0.61));
    path.path.push_back(turnRight(0.61));
    path.path.push_back(goStraight(0.61));
    path.path.push_back(turnLeft(1.22));
    path.path.push_back(goStraight(0.61));
    path.path.push_back(turnRight(1.22));
    path.path.push_back(goStraight(0.61));
    path.path.push_back(turnLeft(0.61));
    path.path.push_back(goStraight(0.61));

    path.path_length = path.path.size();
    return path;
}

// Main function
int main(int argc, char** argv) {
    int numTimes = 1;
    if (argc > 1) {
        numTimes = std::atoi(argv[1]);
    }

    std::cout << "Commanding robot to drive through maze " << numTimes << " times.\n";


    // Generate path
    path2D_t path = generatePath(numTimes);

    // Publish path to LCM
    lcm::LCM lcmInstance(MULTICAST_URL);
    std::cout << "Publishing to: " << CONTROLLER_PATH_CHANNEL << std::endl;

    // print out path
    std::cout << "Generated Path:" << std::endl;
    for (size_t i = 0; i < path.path.size(); ++i) {
        std::cout << "Waypoint " << i << ": "
                << "x=" << path.path[i].x << ", "
                << "y=" << path.path[i].y << ", "
                << "theta=" << path.path[i].theta * 180.0 / M_PI << " degrees"
                << std::endl;
    }

    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
