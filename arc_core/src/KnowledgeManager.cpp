#include "../include/KnowledgeManager.h"
#include "arc_msgs/BotInfo.h"
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

  // Get information about this specific robot
  if (local_handle.getParam("robot_id", robot_id))
    ROS_INFO("Set parameter: robot_id=%d", robot_id);
  else
    ROS_ERROR("Failed to get parameter 'robot_id'");

  local_handle.param("team_id", team_id, 0);
  ROS_INFO("Set parameter: team_id=%d", team_id);
  local_handle.param("role", role, 0);
  ROS_INFO("Set parameter: role=%d", role);

  team_timestamp = ros::Time::now();

  // Advertise services
  attributes_server = this->local_handle.advertiseService("attributes", 
      &KnowledgeManager::attributesCB, this);

  // Advertise publishers
  bot_info_pub = local_handle.advertise<arc_msgs::BotInfo>("bot_info", MAX_QUEUE_SIZE);

  info_timer = local_handle.createTimer(ros::Duration(info_pub_period), 
      &KnowledgeManager::publishInfo, this);
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

bool KnowledgeManager::attributesCB(arc_msgs::Attributes::Request &req,
    arc_msgs::Attributes::Response &res)
{
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

void KnowledgeManager::publishInfo(const ros::TimerEvent& event)
{
  ROS_INFO("Publishing robot information");

  // Prepare the message
  arc_msgs::BotInfo info;
  info.robot_id = robot_id;
  info.team_id = team_id;
  info.role = role;
  info.suitability = role_suitability;

  bot_info_pub.publish(info);
}
