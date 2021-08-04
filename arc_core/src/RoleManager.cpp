#include "../include/RoleManager.h"
#include "arc_msgs/BotInfoRequest.h"
#include "arc_msgs/CurrentTeam.h"
#include "arc_msgs/IdealTeam.h"
#include "arc_msgs/Roles.h"
#include "arc_msgs/SetRole.h"
#include "arc_msgs/TaskSuitability.h"
#include <std_msgs/Int32.h>
#include <unique_id/unique_id.h>

#define MAX_QUEUE_SIZE 1000 
#define DEFAULT_ROS_RATE 10

RoleManager::RoleManager()
{
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("role_manager");
  this->global_handle = global_handle;
  this->local_handle = local_handle;

  // Setup service clients
  roles_client = global_handle.serviceClient<arc_msgs::Roles>("knowledge_manager/roles");
  current_team_client = global_handle.serviceClient<arc_msgs::CurrentTeam>(
      "knowledge_manager/current_team");
  ideal_team_client = global_handle.serviceClient<arc_msgs::IdealTeam>(
      "knowledge_manager/ideal_team");
  bot_info_client = global_handle.serviceClient<arc_msgs::BotInfoRequest>(
      "knowledge_manager/bot_info_request");

  // Setup publishers
  update_role_pub = global_handle.advertise<arc_msgs::SetRole>("knowledge_manager/update_role",
      MAX_QUEUE_SIZE);
  update_team_pub = global_handle.advertise<uuid_msgs::UniqueID>("knowledge_manager/update_team",
      MAX_QUEUE_SIZE);

  role_check_timer = local_handle.createTimer(role_check_period, &RoleManager::roleCheckCB, this);
}

void RoleManager::run()
{
  ros::Rate rate(DEFAULT_ROS_RATE);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void RoleManager::roleCheckCB(const ros::TimerEvent& event)
{
  arc_msgs::CurrentTeam current_team;
  arc_msgs::BotInfoRequest bot_info_req;
  arc_msgs::Roles roles;

  bool info_ret = fetchInformation(current_team, bot_info_req, roles);
  ROS_ASSERT(info_ret);

  // Determine team membership
  //TODO Currently a new team ID is generated every role check if the robot is in a team of
  //     one.  May want to correct this in the future
  if (current_team.response.team.empty())
  {
    auto new_team{ unique_id::fromRandom() };
    update_team_pub.publish(unique_id::toMsg(new_team));
  }
    
  // Determine role
  arc_msgs::Role best_role;
  int best_role_suitability{ std::numeric_limits<int>::min() / 2 };
  int best_role_weighting{ std::numeric_limits<int>::min() / 2 };
  
  for (auto& role : roles.response.roles)
  {
    int suitability{ roleSuitability(role) };
    int importance_weighting{ roleImportance(role, current_team.response, suitability) };
  
    if (suitability + importance_weighting > best_role_suitability + best_role_weighting)
    {
      best_role = role;
      best_role_suitability = suitability;
      best_role_weighting = importance_weighting;
    }
  }
  
  arc_msgs::SetRole set_role;
  set_role.role_id = best_role.role_id;
  set_role.suitability = best_role_suitability;
  update_role_pub.publish(set_role);
}

int RoleManager::roleImportance(arc_msgs::Role& role, arc_msgs::CurrentTeamResponse& current_team,
    int suitability)
{
  long like_agents{ std::count_if(current_team.team.begin(), current_team.team.end(),
                                 [role] (arc_msgs::BotInfo& team_member) 
                                 {
                                   return role.role_id == team_member.role;
                                 }) };
  
  // Get the ideal team proportions for this role
  arc_msgs::IdealTeam ideal_team;
  std::size_t role_index;
  if (!ideal_team_client.call(ideal_team))
  {
    ROS_ERROR("Failed to call service ideal_team");
    return -1;
  }
  ROS_ASSERT(!ideal_team.response.role_ids.empty());
  for (role_index = 0; role_index < ideal_team.response.role_ids.size() && 
      ideal_team.response.role_ids[role_index] != role.role_id; ++role_index) 
  {
  }
  int minimum{ ideal_team.response.minimums[role_index] };
  int maximum{ ideal_team.response.maximums[role_index] };
  int importance{ ideal_team.response.importances[role_index] };

  // Calculate the importance weighting
  int weighting{ 0 };
  if (like_agents < minimum)
  {
    weighting = (minimum - like_agents) / minimum * importance;
  }
  else if (like_agents >= minimum && like_agents < maximum)
  {
    weighting = (maximum - like_agents - minimum) / (maximum - minimum) / 2 * importance;
  }
  else if (like_agents >= maximum)
  {
    // Determine if there is another agent on our team who has the same role but lower suitability
    if (std::count_if(current_team.team.begin(), current_team.team.end(),
         [role, suitability] (arc_msgs::BotInfo& team_member)
         {
           return role.role_id == team_member.role && team_member.suitability < suitability;
         }) )
    {
      weighting = importance;
    } 
  }

  return weighting;
}

//TODO Since attributes are constant it may make sense to cache suitabilities for each role
//  either here or in the knowledge manager
int RoleManager::roleSuitability(arc_msgs::Role& role)
{
  int suitability = 0;
  
  for (int i{ 0 }; i < role.tasks.size(); ++i)
  {
    auto& task{ role.tasks[i] };
    double task_weight{ role.weights[i] };
    arc_msgs::TaskSuitability task_suitability;

    // Check the cache of suitability clients
    if (task_suitability_clients.count(task))
    {
      task_suitability_clients[task].call(task_suitability);
      suitability += static_cast<int>(task_suitability.response.suitability * task_weight);
    }
    else
    {
      //TODO Task suitability clients should probably be setup in constructor
      ros::ServiceClient task_suitability_client = 
        global_handle.serviceClient<arc_msgs::TaskSuitability>(task + "_server/suitability");
     
      if (task_suitability_client.exists())
      {
        task_suitability_client.call(task_suitability);
        suitability += static_cast<int>(task_suitability.response.suitability * task_weight);

        task_suitability_clients[task] = task_suitability_client;
      }
      else
      {
        ROS_WARN("The tasks %s does not exist", task.c_str());
      }
    }
  }

  return suitability;
}

bool RoleManager::fetchInformation(arc_msgs::CurrentTeam& current_team, 
    arc_msgs::BotInfoRequest& bot_info_req, arc_msgs::Roles& roles)
{
  // Fetch the neccessary information
  if (!current_team_client.call(current_team))
  {
    ROS_ERROR("Failed to call service current_team");
    return false;
  }
  if (!bot_info_client.call(bot_info_req))
  {
    ROS_ERROR("Failed to call service bot_info_request");
    return false;
  }
  if (!roles_client.call(roles))
  {
    ROS_ERROR("Failed to call service role");
    return false;
  }
  return true;
}
