#ifndef PFMSCONNECTOR_H
#define PFMSCONNECTOR_H
#include <string>
#include <vector>

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>

#include "pfms_types.h"
#include "cell.h"

////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
//#include "tf/transform_datatypes.h"

using std::string;

class PfmsConnector
{
public:

    /*! @brief Construtor takes platform type, used to set up connections to ROS nodes
     *
     *  Will establish the communication to ROS
     *  Should be created to establish communication and reused while needing to communicate to the platform
     *  @param type - the platform type
     */
    PfmsConnector(pfms::PlatformType type);

    ~PfmsConnector();

    /*! @brief Send Ackerman command to ROS, non-blocking call, repeated seq numbers are ignored
     *
     *  @param cmd - command to be sent
     */
    void send(pfms::commands::Ackerman cmd);

    /*! @brief Send Quadcopter command sent to ROS, non-blocking call, repeated seq numbers are ignored
     *
     *  @param cmd - command to be sent
     */
    void send(pfms::commands::Quadcopter cmd);

    /*! @brief Send SkidSteer command sent to ROS, non-blocking call, repeated seq numbers are ignored
     *
     *  @param cmd - command to be sent
     */
    void send(pfms::commands::SkidSteer cmd);

    /*! @brief Send Point to ROS topic /distance, non-blocking call
     *
     *  @param point - the point to be sent 
     */
    void send(pfms::geometry_msgs::Point point);

    /*! @brief Send Goal for visualisation in RVIZ via ROS marker array, non-blocking call
     *
     *  @param goal - the goal to be visualised, the seq number shown on goal as text, if reptaeted seq number it is ignored
     */
    void send(pfms::geometry_msgs::Goal goal);

    /*! @brief Send Cell for visualisation in RVIZ via ROS marker array, non-blocking call
     *
     *  @param goal - the cell to be visualised, the seq number shown on goal as text
     *  @note The sequence numbers will increase with each call, while cell is 2D object a cube of 0.01m height is visualised
     */
    void send(pfms::Cell cell);


    /*! @brief Send platform status to ROS, non-blocking call
     *
     *  @param status - Only responds to UAV status, actions only take-off and landing status
     */
    void send(pfms::PlatformStatus status);

    /*! @brief Reads LaserScan of platform from ROS, blocking call, will
     * be suspended until read
     *
     *  @param laserScan - the LaserScan from the pipe 
     *  @return true indicates is topic received sucsesfully,
     * false timeout occured
     */
    bool read(pfms::sensor_msgs::LaserScan & laserScan);

    /*! @brief Reads Sonar of platform from ROS, blocking call, will
     * be suspended until data read or timeout (3s)
     *
     *  @param scan - the Sonar reading from the pipe (read/send)
     *  @return true indicates is command written sucsesfully,
     * false timeout occured 
     */
    bool read(pfms::sensor_msgs::Sonar & scan);

    /*! @brief Reads Odometry of platforms from ROS depending on type
     *  blocking call, will be suspended until commend written or timeout (3s)
     *
     *  @param odo - the Odometry from the pipe (read)
     *  @param type - the Platform type 
     *  @return true indicates command written sucsesfully,
     * false timeout occured
     */
    bool read(pfms::nav_msgs::Odometry& odo);

private:
    /*! @brief Reads Ackerman Odometry of platform from ROS, blocking call, will be suspended until commend written or timeout (3s)
     *
     *  @param odo - the Odometry from the pipe (read)
     *  @return true indicates command written sucsesfully,
     * false timeout occured
    */
    bool readAckermanOdometry(pfms::nav_msgs::Odometry& msg);
    /*! @brief Reads SkidSteer Odometry of platform from ROS, blocking call, will be suspended until commend written or timeout (3s)
    *
    *  @param odo - the Odometry from the pipe (read)
    *  @return true indicates command written sucsesfully,
    * false timeout occured
    */
    bool readSkidSteerOdometry(pfms::nav_msgs::Odometry& msg);
    /*! @brief Reads Quadcopter Odometry of platform from ROS, blocking call, will be suspended until commend written or timeout (3s)
    *  *
    *  @param odo - the Odometry from the pipe (read)
    *  @return true indicates command written sucsesfully,
    * false timeout occured
    */
    bool readQuadcopterOdometry(pfms::nav_msgs::Odometry& msg);
    /*! @brief Callback function for handling Ackerman odometry messages
     *
     *  @param msg - the odometry message
     */
    void ackermanOdoCallback(const nav_msgs::msg::Odometry& msg);
    /*! @brief Callback function for handling SkidSteer odometry messages
     *
     *  @param msg - the odometry message
     */
    void skidSteerOdoCallback(const nav_msgs::msg::Odometry& msg);
    /*! @brief Callback function for handling Quadcopter odometry messages
     *
     *  @param msg - the odometry message
     */
    void quadcopterOdoCallback(const nav_msgs::msg::Odometry& msg);
    /*! @brief Callback function for handling LaserScan messages
    *
    *  @param msg - the LaserScan message
    */
    void laserCallback(const sensor_msgs::msg::LaserScan& msg);
    /*! @brief Callback function for handling Sonar messages
     *
     *  @param msg - the Sonar message
     */

    void sonarCallback(const sensor_msgs::msg::LaserScan& msg);
    /*! @brief Main loop for the connector, runs in a separate thread, continuously checks for new data and processes it
     *
     *  This function is responsible for checking if new data has been received from ROS and processing it accordingly. It runs in a separate thread to ensure that the main thread is not blocked while waiting for new data.
     */
    void spin(void);
    /*! @brief Helper function to add a marker to the marker array for visualisation in RVIZ
     *
     *  @param x - the x coordinate of the marker
     *  @param y - the y coordinate of the marker
     *  @param side - the side length of the marker (default is 1.0)
     *  @param seq - the sequence number for the marker, used for tracking changes
     *  @param type - the type of cell being visualised, used to set colour of marker
     *  @param isSeqSet - flag to indicate if sequence number has been set manually (used in constructor with parameters)
     */
    void addMarker(double x, double y, unsigned int seq);
    /*! @brief Helper function to add a marker to the marker array for visualisation in RVIZ, overloaded to include side length and type for visualising cells
     *
     *  @param x - the x coordinate of the marker
     *  @param y - the y coordinate of the marker
     *  @param side - the side length of the marker (default is 1.0)
     *  @param seq - the sequence number for the marker, used for tracking changes
     *  @param type - the type of cell being visualised, used to set colour of marker
     *  @param isSeqSet - flag to indicate if sequence number has been set manually and will therefore be displayed on marker, used in constructor with parameters to indicate if seq number should be displayed
     */
    void addMarker(double x, double y, double side, unsigned int seq, pfms::cell::State type, bool isSeqSet);
    /*! @brief Function to make the quadcopter land
     */
    void quadcopterLand(void);
    /*! @brief Function to make the quadcopter take off
     */
    void quadcopterTakeOff(void);
private:

    struct sync_details{
        std::condition_variable cv; //<! convar to synch getting and reading data
        std::atomic<bool> ready;    //<! Indicates if new  data has been recieved
        std::mutex mtx;             //<! mutex to protect data
    };

    pfms::nav_msgs::Odometry ackermanOdo_; //<! Local record of odometry;
    pfms::nav_msgs::Odometry skidSteerOdo_; //<! Local record of odometry;
    pfms::nav_msgs::Odometry quadcopterOdo_; //<! Local record of odometry;

    pfms::sensor_msgs::Sonar sonar_; //<! Local record of sonar reading;
    pfms::sensor_msgs::LaserScan laser_; //<! Local record of laser scan;
    std::vector<std::thread> threads_; // We add threads onto a vector here to be able to terminate then in destructor
    std::atomic<bool> running_; // We use this to indicate threads shoudl still be running

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ackermanOdoSub_; //<! Subscription for Ackerman odometry messages
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr skidSteerOdoSub_; //<! Subscription for SkidSteer odometry messages
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr quadcopterOdoSub_; //<! Subscription for Quadcopter odometry messages
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_; //<! Subscription for LaserScan messages
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonarSub_; //<! Subscription for Sonar messages, using LaserScan message type as Sonar is not a standard ROS message type and this allows us to reuse the range field for sonar reading

    sync_details ackermanOdoSync_; //<! Sync details for Ackerman odometry, used to synchronise getting and reading data
    sync_details skidSteerOdoSync_; //<! Sync details for SkidSteer odometry, used to synchronise getting and reading data
    sync_details quadcopterOdoSync_; //<! Sync details for Quadcopter odometry, used to synchronise getting and reading data
    sync_details sonarSync_; //<! Sync details for Sonar, used to synchronise getting and reading data
    sync_details laserSync_; //<! Sync details for LaserScan, used to synchronise getting and reading data

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vizPub_; //<! Publisher for visualisation in RVIZ, used to publish marker array
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr quadcopterCmdPub_; //<! Publisher for quadcopter commands, used to publish Twist messages for quadcopter control
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr skidSteerCmdPub_; //<! Publisher for skid steer commands, used to publish Twist messages for skid steer control
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr quadcopterTakeOffPub_; //<! Publisher for quadcopter take off, used to publish Empty messages to trigger quadcopter take off
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr quadcopterLandPub_; //<! Publisher for quadcopter landing, used to publish Empty messages to trigger quadcopter landing
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttlePub_; //<! Publisher for throttle commands, used to publish Float64 messages for throttle control of Ackerman vehicle
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brakePub_; //<! Publisher for brake commands, used to publish Float64 messages for brake control of Ackerman vehicle
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steeringPub_; //<! Publisher for steering commands, used to publish Float64 messages for steering control of Ackerman vehicle 
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pointPub_; //<! Publisher for Point messages, used to publish Point messages for visualisation or other purposes

    rclcpp::Node::SharedPtr node_; //<! ROS node, used to create publishers and subscribers, and to handle communication with ROS

    int marker_counter_; //<! Counter for marker sequence numbers, used to assign unique sequence numbers to markers for tracking changes in visualisation
    unsigned int seq_; //!< Sequence number for tracking changes in visualisation, used to assign unique sequence numbers to markers for tracking changes in visualisation
    visualization_msgs::msg::MarkerArray marker_array_;     //!< Marker array for visualisation in RVIZ, used to store markers for visualisation and publish them to ROS

    bool quadcopterFlying_; //!< Flag to indicate if quadcopter is currently flying, used to prevent take off command being sent when already flying or landing command being sent when already landed
    unsigned int ugvSeq_; //!< Sequence number for tracking changes in visualisation of UGV, used to assign unique sequence numbers to markers for tracking changes in visualisation of UGV

    std::chrono::seconds timeout; //!< Timeout duration for blocking read calls, used to specify how long to wait for new data before timing out and returning false
    int connectionFailureCount_; //!< Counter for connection failures, used to track how many times a connection failure has occurred when trying to read data from ROS, can be used for debugging or to trigger reconnection attempts if desired

    pfms::PlatformType platformType_;//!< The type of platform we are using, used to set connections to ROS nodes
};

#endif // PFMSCONNECTOR_H
