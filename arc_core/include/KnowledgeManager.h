/**
* CLASS: KnowledgeManager
* DATE: 29/06/21
* AUTHOR: Aaron Nedelec
* DESCRIPTION: Manages operational knowledge about the robot's attributes and those of other
*   encountered robots.
*/

#ifndef ARC_CORE_KNOWLEDGEMANAGER_H
#define ARC_CORE_KNOWLEDGEMANAGER_H

#include "ros/ros.h"
#include "arc_msgs/Attributes.h"
#include <list>

class KnowledgeManager
{
private:
  // Handles for this node
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle;

  /// How often in seconds to publish bot info
  const int info_pub_period = 15;

  
  /* PHYSICAL ATTRIBUTES */

  /// Robot's method of locomotion, 0: wheeled, 1: tracked
  int locomotion;

  /// Robot's width in meters
  float width;

  /// Robot's length in meters
  float length;

  /// Robot's expendability
  float expendability;

  /// Whether the robot has a debris remover
  bool debris_remover;

  /// How many markers the robot is carrying
  int marker_count;


  /* COMPUTATIONAL ATTRIBUTES */

  /// Whether the robot has a victim tracker
  bool victim_tracker;

  /// Whether the robot can find frontiers
  bool frontier_finder;

  /// Whether the robot can maintain a team map
  bool maintain_team_map;

  /// Whether the robot can assign tasks
  bool assign_tasks;

  /// Whether the robot has a path planner
  bool planner;


  /* SENSORY ATTRIBUTES */

  /// What kind of victim sensor the bot has
  /// 0: none, 1: basic, 2: advanced
  int victim_sensor;

  /// Whether the bot has a robot sensor
  bool robot_sensor;

  /// How many sonar sensors the robot has
  int sonar_sensors;

  /// Range of the sonar sensor
  float sonar_range;

  /// Whether the robot has a laser rangefinder
  bool laser_rangefinder;

  /// Whether the robot has a victim marker detector
  bool victim_marker_detector;


  /* ENCOUNTERED ROBOTS */

  /// Stores information on a known robot
  struct KnownBot
  {
    int robot_id;
    int team_id;
    std::string role;
    double role_suitability;
    ros::Time timestamp;
  };

  /// Stores knowledge on encountered robots
  std::list<KnownBot> known_robots;

  /* ROBOT INFORMATION */

  /// The robot's unique ID number
  int robot_id;

  /// The robot's current team ID
  int team_id;

  /// Timestamp of when we last heard from our team
  ros::Time team_timestamp;

  /// The robot's current role
  /// 0: team leader, 1: coordinator/explorer, 2: explorer, 3: debris remover
  int role;

  /// The robot's suitability to the current role
  double role_suitability;

  /* SERVICES */

  /// Service to retrieve attribute information
  ros::ServiceServer attributes_server;

  /// Publishes info about the bot
  ros::Publisher bot_info_pub;

  /// Timer to periodically publish self info
  ros::Timer info_timer;


  /* SETUP */

  void getPhysAttributes();

  void getCompAttributes();

  void getSensoryAttributes();

public:

  KnowledgeManager();

  /**
   * Main loop
   */
  void run();

  /**
   * Requests to retrieve the robot's attributes go here
   * @param req The request sent to the knowledge manager
   * @param res The knowledge manager's response to the request
   */
  bool attributesCB(arc_msgs::Attributes::Request &req, 
      arc_msgs::Attributes::Response &res);

  /**
   * Timer callback to publish self info
   */
  void publishInfo(const ros::TimerEvent& event);
};

#endif //ARC_CORE_KNOWLEDGEMANAGER_H
