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
#include "arc_msgs/BotInfo.h"
#include "arc_msgs/BotInfoRequest.h"
#include "arc_msgs/CurrentTeam.h"
#include <unique_id/unique_id.h>
#include <vector>

class KnowledgeManager
{
public:
  using id_t = boost::uuids::uuid;

private:
  // Handles for this node
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle;

  /// How often in seconds to publish bot info
  const int info_pub_period = 15;

  /// How long to remember information
  const ros::Duration stale_duration = ros::Duration(3 * 60);
  
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

  /**
   * Stores information on an encountered robot
   */
  struct KnownBot
  {
    id_t robot_id;
    int team_id;
    int role;
    int role_suitability;
    ros::Time timestamp;
  };

  /**
   * Stores knowledge on all encountered robots
   */
  std::vector<KnownBot> known_bots;

  /* ROBOT INFORMATION */

  /// The robot's unique ID number
  id_t robot_id;

  /// The robot's current team ID
  int team_id;

  /// Timestamp of when we last heard from our team
  ros::Time team_timestamp;

  /// The robot's current role
  /// 0: team leader, 1: coordinator/explorer, 2: explorer, 3: debris remover
  int role;

  /// The robot's suitability to the current role
  int role_suitability;

  /* COMMUNICATION  */

  /// Service to retrieve attribute information
  ros::ServiceServer attributes_server;

  /**
   * Service to provide information on the robot
   */
  ros::ServiceServer bot_info_server;

  /**
   * Service to provide current team information
   */
  ros::ServiceServer current_team_server;

  /// Publishes info about the bot
  ros::Publisher bot_info_pub;

  /// Subcribes to information coming from other robots
  ros::Subscriber bot_info_sub;

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
   * Fullfills requests to retrieve current information on the bot
   * @param req The request sent to the knowledge manager
   * @param res The knowledge manager's response to the request
   */
  bool botInfoCB(arc_msgs::BotInfoRequest::Request& req,
      arc_msgs::BotInfoRequest::Response& res);

  /**
   * Fullfills requests to retrieve the current team information
   * @param req Request sent to the server
   * @param res Server's response to the request
   */
  bool currentTeamCB(arc_msgs::CurrentTeam::Request& req,
      arc_msgs::CurrentTeam::Response& res);

  /**
   * Timer callback to publish self info
   */
  void publishInfo(const ros::TimerEvent& event);

  /**
   * Updates our information about other robots
   */
  void updateInfo(const arc_msgs::BotInfo& info);
};

#endif //ARC_CORE_KNOWLEDGEMANAGER_H
