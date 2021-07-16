#include "MinimalRecruitmentManager.h"
#include <ros/ros.h>

#define DEFAULT_RATE 10
#define MAX_QUEUE_SIZE 1000

MinimalRecruitmentManager::MinimalRecruitmentManager() : 
  local_handle("recruitment_manager"),
  task_coordination_server(global_handle, "task_coordination", false)
{
  this->task_requests_sub = this->global_handle.subscribe("communication_manager/task_requests", 
      MAX_QUEUE_SIZE, &MinimalRecruitmentManager::task_request_cb, this);
  this->task_request_pub = this->global_handle.advertise<arc_msgs::TaskRequest>(
      "task_handler/task_requests", MAX_QUEUE_SIZE);

  // Setup task coordination
  task_coordination_server.registerGoalCallback(
      boost::bind(&MinimalRecruitmentManager::taskCoordinationCB, this));
  task_coordination_server.registerPreemptCallback(
      boost::bind(&MinimalRecruitmentManager::taskCoordPreemptCB, this));
  
  task_response_pub = global_handle.advertise<arc_msgs::TaskResponse>(
      "communication_manager/task_response_out", MAX_QUEUE_SIZE);
  task_confirmation_sub = global_handle.subscribe("communication_manager/task_confirmations",
      MAX_QUEUE_SIZE, &MinimalRecruitmentManager::taskConfirmationCB, this);

  task_coordination_server.start();
}

void MinimalRecruitmentManager::process() 
{
  ros::Rate rate(DEFAULT_RATE);

  while(ros::ok()) 
  {
    ros::spinOnce();
  }
}

void MinimalRecruitmentManager::task_request_cb(const arc_msgs::TaskRequest &req) 
{
  ROS_INFO("Received task request. Sending it over to the task_handler!");
  this->task_request_pub.publish(req);
}

void MinimalRecruitmentManager::taskCoordinationCB()
{
  ROS_INFO("Sending task response");
  arc_msgs::TaskResponse response = task_coordination_server.acceptNewGoal()->response;
  task_id = response.task_id;
  task_response_pub.publish(response);
}

void MinimalRecruitmentManager::taskCoordPreemptCB()
{
  ROS_INFO("task_coordination: Preempted");
  task_coordination_server.setPreempted();
}

void MinimalRecruitmentManager::taskConfirmationCB(const arc_msgs::TaskConfirmation& con)
{
  if (!task_coordination_server.isActive() || con.task_id != task_id)
    return;

  task_confirmation.confirmation = con.confirmation;
  task_coordination_server.setSucceeded(task_confirmation);
}
