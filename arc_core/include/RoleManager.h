/**
 * CLASS: RoleManager
 * DATE: 22/07/21
 * AUTHOR: Aaron Nedelec
 * DESCRIPTION: Manages our role by performing regular role checks.
 */

#ifndef ARC_CORE_ROLEMANAGER_H
#define ARC_CORE_ROLEMANAGER_H

#include "arc_msgs/CurrentTeam.h"
#include "arc_msgs/BotInfoRequest.h"
#include "arc_msgs/Roles.h"
#include <ros/ros.h>

class RoleManager
{
private:
  const ros::Duration role_check_period{ ros::Duration(30) };
  
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle;

  /// Client for requesting the defined roles from the knowledge manager
  ros::ServiceClient roles_client;

  ros::ServiceClient current_team_client;

  ros::ServiceClient ideal_team_client;

  ros::ServiceClient bot_info_client;
 
  /// Stores the suitability clients
  std::map<std::string, ros::ServiceClient> task_suitability_clients;

  ros::Timer role_check_timer;
  
  // Publishers
  ros::Publisher update_role_pub;

  ros::Publisher update_team_pub;

  /**
   * Fetches the necessary information for a role check
   */
  bool fetchInformation(arc_msgs::CurrentTeam& current_team, arc_msgs::BotInfoRequest& bot_info_req,
      arc_msgs::Roles& roles);

  /**
   * Calculates the robot's suitability to fill a role
   * @param role The role to calculate suitability for
   */
  int roleSuitability(arc_msgs::Role& role);

  /**
   * Calculate the role importance weighting of the given role
   * @param role The role to calculate the weighting for
   * @param current_team The current known team composition
   * @param suitability The robot's suitability to fill the role
   * @return Returns the role importance weighting of the role or -1 if an error occurs
   */
  int roleImportance(arc_msgs::Role& role, arc_msgs::CurrentTeamResponse& current_team, 
      int suitability);
  
public:
  RoleManager();

  void run();

  /**
   * Performs a role check
   */
  void roleCheckCB(const ros::TimerEvent& event);
};

#endif
