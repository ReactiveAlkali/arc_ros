#include <algorithm>
#include <string>
#include "../include/KnowledgeManager.h"
#include "arc_msgs/BotInfo.h"
#include "arc_msgs/Attributes.h"
#include "ros/ros.h"

#define MAX_QUEUE_SIZE 1000
#define DEFAULT_ROS_RATE 10

KnowledgeManager::KnowledgeManager()
{
  // Setup node handles
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("knowledge_manager");
  this->global_handle = global_handle;
  this->local_handle = local_handle;

  ROS_INFO("Setting up Knowledge Managers");

  // Read attribute parameters
  getPhysAttributes();
  getCompAttributes();
  getSensoryAttributes();

  local_handle.param("role", role, 0);
  ROS_INFO("Set parameter: role=%d", role);

  team_timestamp = ros::Time::now();
  getIDs();
  getRoles();
  getIdealTeam();

  // TODO Role suitability calculation
  role_suitability = 100;

  // Advertise services
  attributes_server = this->local_handle.advertiseService("attributes", 
      &KnowledgeManager::attributesCB, this);
  bot_info_server = local_handle.advertiseService("bot_info_request", &KnowledgeManager::botInfoCB,
      this);
  current_team_server = local_handle.advertiseService("current_team", 
      &KnowledgeManager::currentTeamCB, this);
  roles_server = local_handle.advertiseService("roles", &KnowledgeManager::rolesCB, this);
  ideal_team_server = local_handle.advertiseService("ideal_team", &KnowledgeManager::idealTeamCB,
      this);

  // Advertise publishers
  bot_info_pub = local_handle.advertise<arc_msgs::BotInfo>("bot_info", MAX_QUEUE_SIZE);

  info_timer = local_handle.createTimer(ros::Duration(info_pub_period), 
      &KnowledgeManager::publishInfo, this);

  bot_info_sub = global_handle.subscribe("communication_manager/bot_information", MAX_QUEUE_SIZE,
      &KnowledgeManager::updateInfo, this);
}

//-------------------------------------
//
// SETUP
//
//-------------------------------------

void KnowledgeManager::getIDs()
{
  std::string id_string{};

  // Get our robot ID
  if (local_handle.getParam("robot_id", id_string) && id_string != "")
  {
    ROS_INFO("Set parameter: robot_id=%s", id_string.c_str());
    robot_id = unique_id::fromHexString(id_string);
  }
  else
  {
    robot_id = unique_id::fromRandom();
    ROS_INFO("Generated unique robot ID: %s", unique_id::toHexString(robot_id).c_str());
  }

  // Get our team ID
  if (local_handle.getParam("team_id", id_string) && id_string != "")
  {
    ROS_INFO("Set parameter: team_id=%s", id_string.c_str());
    team_id = unique_id::fromHexString(id_string);
  }
  else
  {
    team_id = unique_id::fromRandom();
    ROS_INFO("Generating unique team ID: %s", unique_id::toHexString(team_id).c_str());
  }
}

void KnowledgeManager::getIdealTeam()
{
  std::string ideal_team_string;

  if (local_handle.getParam("ideal_team", ideal_team_string))
    ROS_DEBUG("Got ideal team string");
  else
    ROS_ERROR("Failed to get ideal team string");

  // Parse the string
  std::stringstream ideal_team_stream(ideal_team_string);
  std::string token;

  while (getline(ideal_team_stream, token, '|'))
  {
    Ideal temp{};
    
    // Extract the ideal team characteristics of a role
    std::stringstream ideal_stream(token);
    getline(ideal_stream, token, ',');
    temp.role_id = std::stoi(token);
    getline(ideal_stream, token, ',');
    temp.minimum = std::stoi(token);
    getline(ideal_stream, token, ',');
    temp.maximum = std::stoi(token);

    ideal_team.push_back(temp);
  }
}

/*
 * Roles are given as a string with the following format:
 *   roleID1:task1,weight1;task2,wieght2|roleID2:task1,weight1
 */
void KnowledgeManager::getRoles()
{
  std::string roles_string;

  if (local_handle.getParam("roles", roles_string))
    ROS_DEBUG("Got roles string");
  else
    ROS_ERROR("Failed to get roles string");

  // Parse the string roles
  std::stringstream roles(roles_string);
  std::string token;

  while (getline(roles, token, '|'))
  {
    Role temp{};

    // Parse the role string
    std::stringstream role(token);
    getline(role, token, ':');
    temp.role_id = std::stoi(token);

    // Extract the task list
    while (getline(role, token, ';'))
    {
      std::stringstream task(token);
      std::string name, weight;
      getline(task, name, ',');
      getline(task, weight, ',');

      temp.expected_tasks[name] = std::stod(weight);
    }

    this->roles.push_back(temp);
  }
}

void KnowledgeManager::getPhysAttributes()
{
  if (local_handle.getParam("locomotion", this->locomotion))
    ROS_INFO("Set parameter: locomotion=%d", this->locomotion);
  else
    ROS_ERROR("Failed to get parameter 'locomotion'");

  if (local_handle.getParam("width", this->width))
    ROS_INFO("Set parameter: width=%f", this->width);
  else
    ROS_ERROR("Failed to get parameter 'width'");

  if (local_handle.getParam("length", this->length))
    ROS_INFO("Set parameter: length=%f", this->length);
  else
    ROS_ERROR("Failed to get parameter 'length'");

  if (local_handle.getParam("expendability", this->expendability))
    ROS_INFO("Set parameter: expendability=%f", this->expendability);
  else
    ROS_ERROR("Failed to get parameter 'expendability'");

  if (local_handle.getParam("debris_remover", this->debris_remover))
    ROS_INFO("Set parameter: debris_remover=%d", this->debris_remover);
  else
    ROS_ERROR("Failed to get parameter 'debris_remover'");

  if (local_handle.getParam("marker_count", this->marker_count))
    ROS_INFO("Set parameter: marker_count=%d", this->marker_count);
  else
    ROS_ERROR("Failed to get parameter 'marker_count'");
}

void KnowledgeManager::getCompAttributes()
{
  if (local_handle.getParam("victim_tracker", this->victim_tracker))
    ROS_INFO("Set parameter: victim_tracker=%d", this->victim_tracker);
  else
    ROS_ERROR("Failed to get parameter 'victim_tracker'");

  if (local_handle.getParam("frontier_finder", this->frontier_finder))
    ROS_INFO("Set parameter: frontier_finder=%d", this->frontier_finder);
  else
    ROS_ERROR("Failed to get parameter 'frontier_finder'");

  if (local_handle.getParam("maintain_team_map", this->maintain_team_map))
    ROS_INFO("Set parameter: maintain_team_map=%d", this->maintain_team_map);
  else
    ROS_ERROR("Failed to get parameter 'maintain_team_map'");

  if (local_handle.getParam("assign_tasks", this->assign_tasks))
    ROS_INFO("Set parameter: assign_tasks=%d", this->assign_tasks);
  else
    ROS_ERROR("Failed to get parameter 'assign_tasks'");

  if (local_handle.getParam("planner", this->planner))
    ROS_INFO("Set parameter: planner=%d", this->planner);
  else
    ROS_ERROR("Failed to get parameter 'planner'");
}

void KnowledgeManager::getSensoryAttributes()
{
  if (local_handle.getParam("victim_sensor", this->victim_sensor))
    ROS_INFO("Set parameter: victim_sensor=%d", this->victim_sensor);
  else
    ROS_ERROR("Failed to get parameter 'victim_sensor'");

  if (local_handle.getParam("robot_sensor", this->robot_sensor))
    ROS_INFO("Set parameter: robot_sensor=%d", this->robot_sensor);
  else
    ROS_ERROR("Failed to get parameter 'robot_sensor'");

  if (local_handle.getParam("sonar_sensors", sonar_sensors))
    ROS_INFO("Set parameter: sonar_sensors=%d", sonar_sensors);
  else
    ROS_ERROR("Failed to get parameter 'sonar_sensors'");

  if (local_handle.getParam("sonar_range", sonar_range))
    ROS_INFO("Set parameter: sonar_range=%f", sonar_range);
  else
    ROS_ERROR("Failed to get parameter 'sonar_range'");

  if (local_handle.getParam("laser_rangefinder", laser_rangefinder))
    ROS_INFO("Set parameter: laser_rangefinder=%d", laser_rangefinder);
  else
    ROS_ERROR("Failed to get parameter 'laser_rangefinder'");

  if (local_handle.getParam("victim_marker_detector", victim_marker_detector))
    ROS_INFO("Set parameter: victim_marker_detector=%d", victim_marker_detector);
  else
    ROS_ERROR("Failed to get parameter 'victim_marker_detector'");
}

void KnowledgeManager::run()
{
  ros::Rate rate(DEFAULT_ROS_RATE);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

/*
 * SERVICE CALLBACKS
 */

bool KnowledgeManager::idealTeamCB(arc_msgs::IdealTeam::Request& req, 
    arc_msgs::IdealTeam::Response& res)
{
  for (auto element : ideal_team)
  {
    res.role_ids.push_back(element.role_id);
    res.minimums.push_back(element.minimum);
    res.maximums.push_back(element.maximum);
  }

  return true;
}


bool KnowledgeManager::rolesCB(arc_msgs::Roles::Request& req, arc_msgs::Roles::Response& res)
{
  ROS_DEBUG("Roles request recieved");

  // Create a role message for each role
  for (auto role : roles)
  {
    arc_msgs::Role role_msg;
    role_msg.role_id = role.role_id;

    // Get the tasks and weights of the role
    for (auto task : role.expected_tasks)
    {
      role_msg.tasks.push_back(task.first);
      role_msg.weights.push_back(task.second);
    }

    res.roles.push_back(role_msg);
  }

  return true;
}

bool KnowledgeManager::attributesCB(arc_msgs::Attributes::Request &req,
    arc_msgs::Attributes::Response &res)
{
  ROS_INFO("Attributes request recieved");

  // Physical
  res.locomotion = locomotion;
  res.width = width;
  res.length = length;
  res.expendability = expendability;
  res.debris_remover = debris_remover;
  res.marker_count = marker_count;

  // Computational
  res.victim_tracker = victim_tracker;
  res.frontier_finder = frontier_finder;
  res.maintain_team_map = maintain_team_map;
  res.assign_tasks = assign_tasks;
  res.planner = planner;

  // Sensory
  res.victim_sensor = victim_sensor;
  res.robot_sensor = robot_sensor;
  res.sonar_sensors = sonar_sensors;
  res.sonar_range = sonar_range;
  res.laser_rangefinder = laser_rangefinder;
  res.victim_marker_detector = victim_marker_detector;
  
  return true;
}

bool KnowledgeManager::botInfoCB(arc_msgs::BotInfoRequest::Request& req,
    arc_msgs::BotInfoRequest::Response& res)
{
  res.info.robot_id = unique_id::toMsg(robot_id);
  res.info.team_id = unique_id::toMsg(team_id);
  res.info.role = role;
  res.info.suitability = role_suitability;

  return true;
}

bool KnowledgeManager::currentTeamCB(arc_msgs::CurrentTeam::Request& req,
    arc_msgs::CurrentTeam::Response& res)
{
  ROS_INFO("Fullfilling current team request");

  // Respond with all known bots who are part of our team and whose information isn't stale
  for (KnownBot bot : known_bots)
  {
    ros::Duration age = ros::Time::now() - bot.timestamp;  // How old the info is
    
    if (bot.team_id == team_id && age < stale_duration)
    {
      arc_msgs::BotInfo bot_info;
      bot_info.robot_id = unique_id::toMsg(bot.robot_id);
      bot_info.team_id = unique_id::toMsg(bot.team_id);
      bot_info.role = bot.role;
      bot_info.suitability = bot.role_suitability;
    
      res.team.push_back(bot_info);
    }
  }

  return true;
}

/*
 * TIMER AND SUBSCRIBER CALLBACKS
 */

void KnowledgeManager::publishInfo(const ros::TimerEvent& event)
{
  //ROS_INFO("Publishing robot information");

  // Prepare the message
  arc_msgs::BotInfo info;
  info.robot_id = unique_id::toMsg(robot_id);
  info.team_id = unique_id::toMsg(team_id);
  info.role = role;
  info.suitability = role_suitability;

  bot_info_pub.publish(info);
}

void KnowledgeManager::updateInfo(const arc_msgs::BotInfo& info)
{
  id_t other_bot_id = unique_id::fromMsg(info.robot_id);
  id_t other_bot_team = unique_id::fromMsg(info.team_id);

  // Find if we already know about this bot
  auto found{ std::find_if(known_bots.begin(), known_bots.end(), 
      [other_bot_id](KnownBot a) {
        return (a.robot_id == other_bot_id);
      }) };

  // Add bot if we don't already know about it or update if we do
  if (found == known_bots.end())
  {
    ROS_INFO("Adding bot %s to our knowledge", unique_id::toHexString(other_bot_id).c_str());
    KnownBot new_bot{ other_bot_id, other_bot_team, info.role, info.suitability, ros::Time::now() };
    known_bots.push_back(new_bot);
  }
  else
  {
    ROS_INFO("Updating info about robot %s", unique_id::toHexString(other_bot_id).c_str());
    found->team_id = other_bot_team;
    found->role = info.role;
    found->role_suitability = info.suitability;
    found->timestamp = ros::Time::now();
  }
}
