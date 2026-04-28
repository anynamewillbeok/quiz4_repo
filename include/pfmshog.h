#ifndef PFMSHOG_H
#define PFMSHOG_H

#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <string>
#include <future>

#include "pfms_types.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <ignition/msgs8/ignition/msgs.hh>
#include <ignition/transport11/ignition/transport/Node.hh>
#include "rcl_interfaces/srv/get_parameters.hpp"
// #include "gazebo_msgs/srv/delete_entity.hpp"  // Not available in Gazebo Fortress
// #include "gazebo_msgs/srv/spawn_entity.hpp"   // Not available in Gazebo Fortress
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class PfmsHog
{
public:
    PfmsHog(pfms::PlatformType platform);
    PfmsHog(pfms::PlatformType platform, bool debug);
    ~PfmsHog();

    std::string mapModelNameToPlatform(pfms::PlatformType platform){
        std::string frame;
        switch(platform) {
          case pfms::PlatformType::ACKERMAN_BLUE:
            frame = "blue_audibot";
            break;
          case pfms::PlatformType::ACKERMAN:
            frame = "orange_audibot";
            break;
          case pfms::PlatformType::SKIDSTEER:
            frame = "husky";
            break;
          case pfms::PlatformType::QUADCOPTER:
            frame = "drone";
            break;
        }
        return frame;
    }    

    /*! @brief Teleport the platform to the location specified in the odometry, the change is instateneous.
     *
     *  @param odo - The odometry, currently yaw is the only orientation used, the velocity supplied is ignored (and will be set to zero)
     */
    bool teleport(pfms::nav_msgs::Odometry odo);


    /*! @brief Teleport a named object to the location specified in the odometry, the change is instateneous.
     *
     *  @param point - The point where it will be teleported, currently yaw is the only orientation used
     *  @param name - The name of the object to be teleported, this is the name used in gazebo
     *  @return false, not possible, at the moment name check not implemented
     */
    bool teleportObject(pfms::geometry_msgs::Point point, std::string name);

    /*! @brief Specify the anticipated goal, the platform  will need to travel through this goal 
     *
     *  @param goal - The goal (point) that needs to be reached
     *  @return goal was reached by platform.
     *  @note Internally the code uses ROS services and another ROS component to check goal has been reached by platform
     */
    bool setGoal(pfms::geometry_msgs::Point goal);

    /*! @brief Specify the anticipated goals, the platform will need to travel through all of these goals in the exact order supplied
     *
     *  @param goals - The goals (points) that needs to be reached, the vector implies the order (from 0th to last element)
     *  @return goals were reached by platform in the exact order supplied
     */
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);


    /*! @brief Check that goals set via @sa writeCommand has been travelled through,
     * checks this via a service call to gazebo_tf odo checker
     *
     *  @param [in|out] - vector containing closest distances to all goals supplied 
     *  @param [in|out] - overall time to reach all goals 
     *  @return all goals were travelled through in supplied order
     */
    bool checkGoalsReached(std::vector<double> &distances, double &total_time);

    /*! @brief Check that goals set via @sa writeCommand has been travelled through (backward compatible)
     * checks this via a service call to odo checker
     *
     *  @param [in|out] - vector containing closest distances to all goals supplied 
     *  @return all goals were travelled through in supplied order
     */
    bool checkGoalsReached(std::vector<double> &distances);

private:
    /*! @brief Main loop for the connector, runs in a separate thread, continuously checks for new data and processes it
     *
     *  This function is responsible for checking if new data has been received from ROS and processing it accordingly. It runs in a separate thread to ensure that the main thread is not blocked while waiting for new data.
     */
    void spin(void);

    /*! @brief Obtain MarkerArray of CUBES from geometry_msgs::Point
    * The markers are reported in world coordinate frames, namespace goals, type CUBE, colour green
    *
    *  @param goals - vector of geometry_msgs::Point
    *  @return
    */
    visualization_msgs::msg::MarkerArray produceMarkerList(std::vector<pfms::geometry_msgs::Point> goals);

    // std::string replaceAll(std::string str, const std::string &from, const std::string &to);

private:

    struct sync_details{
        std::condition_variable cv; //<! convar to synch getting and reading data
        std::atomic<bool> ready;    //<! Indicates if new  data has been recieved
        std::mutex mtx;             //<! mutex to protect data
    };


    pfms::PlatformType platform_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr clientParams_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clientGoals_;

    void response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    void responseParam_callback(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future);
    bool service_done_ = false; // inspired from action client c++ code

    bool debug_;/*!< bool indicating if debug information to be provided */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vizPub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uavCmdPub_;//<! Publisher to stop quadcopter
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goalPub_;//<! Publisher to stop quadcopter
    std::vector<std::thread> threads_; // We add threads onto a vector here to be able to terminate then in destructor
    std::atomic<bool> running_; // We use this to indicate threads shoudl still be running
    unsigned int markerCount_; //<! Counter to assign unique IDs to markers, this is needed for visualization in gazebo, otherwise it will not update the position of the marker but create a new one instead

    bool goalSuccess_;
    std::vector<double> distances_; // <! Distances to goals, used to be able to report how close the platform is to the goal when it is not reached yet
    std::atomic<double> total_time_; //<! Time to reach goals, used to be able to report time to reach goals when they are reached
    std::atomic<bool> checkedGoals_; // We use this to indicate threads shoudl still be running
    sync_details goals_sync_; //<! Synchronization for goal checking

    bool serviceTeleport_done_ = false; // inspired from action client c++ code
    std::atomic<bool> paramDone_ ; // Not available in Gazebo Fortress
    
    bool serviceDelete_done_ = false; // inspired from action client c++ code
    std::atomic<bool> deleteDone_ ; // Not available in Gazebo Fortress

    bool serviceSpawn_done_ = false; // inspired from action client c++ code
    std::atomic<bool> spawnDone_ ;  // Not available in Gazebo Fortress

    std::string robotDescription_; //<! This is the robot description, used to check if the model exists in gazebo when teleporting objects, this is obtained from the parameter server

};

#endif // PFMSHOG_H
