#ifndef AUDI_H
#define AUDI_H

#include "pfms_types.h"

/*!
 *  \brief     Audi Class
 *  \details
 *  This class implements computing steering to drive towards a goal from current odometry
 *  It also provides capability to compute distance and time from origin to goal
 *  \author    Alen Alempijevic
 *  \version   1.00-1
 *  \date      2022-04-15
 *  \pre       none
 *  \bug       none reported as of 2022-04-15
 *  \warning   students MUST NOT change this class (the header file)
 */


class Audi
{
public:
  //Default constructor should set all attributes to default audi settings
  Audi();
  ~Audi();

  /**
  Checks whether the platform can travel between origin and destination via constant steering angle
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] goal The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @param[in|out] pathPoints Optional pointer to vector of points along the path. If provided (not nullptr), will be populated with points along the arc 
  from origin to goal (maintaining the same z height as origin). If not provided (nullptr), no points will be generated. 
  The number of points generated is based on the distance to the goal, with approximately one point per 0.5 meters, up to a maximum of 10 points.
  @return bool indicating the platform can reach the destination from origin supplied
  */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose,
                                 std::vector<pfms::geometry_msgs::Point>* pathPoints = nullptr);

  /**
  Checks whether the platform can travel between origin and destination via constant steering angle
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] goal The destination point for the platform
  @param[in|out] steering The steering needed to reach the goal (constant steering)
  @param[in|out] dist The distance along the arc travelled to the goal
  @return bool indicating the platform can reach the goal from odo supplied
  */
  bool computeSteering(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                      double& steering,double& dist);

private:
  pfms::PlatformType type_; //<! The platform type, used to set the parameters for the kinematic model
  const double steering_ratio_; //<! The steering ratio of the platform, used to convert between steering angle and wheel angle
  const double lock_to_lock_revs_; //<! The number of revolutions from lock to lock, used to calculate the max steering angle
  const double max_steer_angle_; //<! The maximum steering angle of the platform, used to check if the goal is reachable and to calculate the steering needed to reach the goal
  const double wheelbase_; //<! The wheelbase of the platform, used to calculate the turning radius and the steering needed to reach the goal
  const double max_break_torque_; //<! The maximum break torque of the platform, used to calculate the deceleration and time needed to reach the goal
  const double steadyStateV_; //<! The steady state velocity of the platform, used to calculate the time needed to reach the goal
  const double deltaD_; //<! The distance step used to calculate the path to the goal, used in the checkOriginToDestination function to calculate the path to the goal and check if it is reachable
  double prevD_; //<! The previous distance to the goal, used in the checkOriginToDestination function to check if the platform is getting closer to the goal or not, if it is not getting closer for a certain number of steps, the goal is considered unreachable

};

#endif // AUDI_H
