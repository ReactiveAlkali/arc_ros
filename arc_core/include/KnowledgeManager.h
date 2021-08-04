/**
* CLASS: KnowledgeManager
* DATE: 29/06/21
* AUTHOR: Aaron Nedelec
* DESCRIPTION: Manages operational knowledge about the robot's attributes and those of other
*   encountered robots.
*/

//TODO Should store a list of valid tasks and pass this information to modules that require it

#ifndef ARC_CORE_KNOWLEDGEMANAGER_H
#define ARC_CORE_KNOWLEDGEMANAGER_H

#include "ros/ros.h"
#include "arc_msgs/Attributes.h"
#include "arc_msgs/BotInfo.h"
#include "arc_msgs/BotInfoRequest.h"
#include "arc_msgs/CurrentTeam.h"
#include "arc_msgs/IdealTeam.h"
#include "arc_msgs/Roles.h"
#include "arc_msgs/SetRole.h"
#include <unique_id/unique_id.h>
#include <std_msgs/Int32.h>
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

  //-----------------------------------
  //
  // ROLES
  //
  //-----------------------------------

  struct Role
  {
    int role_id;                                /// The ID number of the role
    std::map<std::string, double> expected_tasks{};  /// The tasks expected of this role
  };
  
  /**
   * All the defined roles
   */
  std::vector<Role> roles;

  /**
   * Get the defined roles during setup
   */
  void getRoles();

  /**
   * Service to provide all the known roles
   */
  ros::ServiceServer roles_server;

  ros::Subscriber update_role_sub;

  //-----------------------------------
  //
  // TEAM
  //
  //-----------------------------------

  // The ideal minumum and maximum robots of a specific role
  struct Ideal
  {
    int role_id;
    int minimum;
    int maximum;
    int importance;
  };

  std::vector<Ideal> ideal_team;

  ros::ServiceServer ideal_team_server;

  /**
   * Gets the ideal team during setup.  The ideal team is given as a string parameter with
   * the following format:  role_id,min,max,importance|role_id,min,max,importance
   */
  void getIdealTeam();

  //-----------------------------------
  //
  // ROBOT INFORMATION
  //
  //-----------------------------------

  /**
   * Stores information on an encountered robot
   */
  struct KnownBot
  {
    id_t robot_id;
    id_t team_id;
    int role;
    int role_suitability;
    ros::Time timestamp;
  };

  /**
   * Stores knowledge on all encountered robots
   */
  std::vector<KnownBot> known_bots;

  /// The robot's unique ID number
  id_t robot_id;

  /// The robot's current team ID
  id_t team_id;

  /// Timestamp of when we last heard from our team
  ros::Time team_timestamp;

  /// The robot's current role
  int role;

  /// The robot's suitability to the current role
  int role_suitability;

  ros::Subscriber update_team_sub;

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

  void getIDs();

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
   * Fullfills requests to retrieve the characteristics of all the known roles 
   * @param req Request sent to the server
   * @param res Our response to the request
   */
  bool rolesCB(arc_msgs::Roles::Request& req, arc_msgs::Roles::Response& res);
  
  /**
   * Fullfills requests to retrieve the definition of the ideal team
   * @param req The request made to the service server
   * @param res Our response to the request
   */
  bool idealTeamCB(arc_msgs::IdealTeam::Request& req, arc_msgs::IdealTeam::Response& res);

  /**
   * Timer callback to publish self info
   */
  void publishInfo(const ros::TimerEvent& event);

  /**
   * Updates our information about other robots
   */
  void updateInfo(const arc_msgs::BotInfo& info);

  /**
   * Updates our team affiliation
   * @param new_team The ID of the new team we are part of
   */
  void updateTeamCB(const uuid_msgs::UniqueID& new_team);
  
  /**
   * Updates our current role
   * @param new_role The ID of our new role
   */
  void updateRoleCB(const arc_msgs::SetRole& new_role);
};

#endif //ARC_CORE_KNOWLEDGEMANAGER_H
