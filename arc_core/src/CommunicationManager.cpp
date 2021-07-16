#include <XmlRpcException.h>
#include "arc_msgs/TaskRequest.h"
#include "arc_msgs/BotInfo.h"
#include "arc_msgs/BotInfoRequest.h"
#include "arc_msgs/TaskConfirmation.h"
#include "../include/CommunicationManager.h"
#include "dynamic_reconfigure/Config.h"

#define MAX_QUEUE_SIZE 1000
#define DEFAULT_ROS_RATE 10

using namespace XmlRpc;

CommunicationManager::CommunicationManager() 
{
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("communication_manager");
  this->global_handle = global_handle;
  this->local_handle = local_handle;

  //reading parameters
  ROS_INFO("Setting up Communication manager");
  //setting up subscribers
  this->incoming_announcements_sub = global_handle.subscribe(
      "wifi_handler/incoming_announcements", MAX_QUEUE_SIZE, 
      &CommunicationManager::process_incoming_announcements_cb, this);
  this->incoming_requests_sub = global_handle.subscribe("wifi_handler/incoming_requests", 
      MAX_QUEUE_SIZE, &CommunicationManager::process_incoming_requests_cb, this);
  this->incoming_responses_sub = global_handle.subscribe("wifi_handler/incoming_responses", 
      MAX_QUEUE_SIZE, &CommunicationManager::process_incoming_responses_cb, this);
  
  base_pose_sub = global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE,
      &CommunicationManager::processBasePoseCB, this);

  //setting up publishers
  this->outgoing_announcements_pub = global_handle.advertise<arc_msgs::WirelessAnnouncement>(
      "wifi_handler/outgoing_announcements", MAX_QUEUE_SIZE);
  this->outgoing_requests_pub = global_handle.advertise<arc_msgs::WirelessRequest>(
      "wifi_handler/outgoing_requests", MAX_QUEUE_SIZE);
  this->outgoing_responses_pub = global_handle.advertise<arc_msgs::WirelessResponse>(
      "wifi_handler/outgoing_responses", MAX_QUEUE_SIZE);

  //set up communications for tasks
  this->task_requests_pub = local_handle.advertise<arc_msgs::TaskRequest>("task_requests", 
      MAX_QUEUE_SIZE);
  task_confirmations_pub = local_handle.advertise<arc_msgs::TaskConfirmation>("task_confirmations",
      MAX_QUEUE_SIZE);
  outgoing_task_response_sub = local_handle.subscribe("task_response_out", MAX_QUEUE_SIZE,
      &CommunicationManager::sendTaskResponse, this);
  outgoing_task_acknowledgement_sub = local_handle.subscribe("task_acknowledgement_out",
      MAX_QUEUE_SIZE, &CommunicationManager::sendTaskAcknowledgement, this);

  // Handling information about robots
  this->bot_info_pub = local_handle.advertise<arc_msgs::BotInfo>("bot_information", 
      MAX_QUEUE_SIZE);
  bot_info_client = global_handle.serviceClient<arc_msgs::BotInfoRequest>(
      "knowledge_manager/bot_info_request");

  /**
   * Populating valid task list
   */
  try 
  {
    XmlRpcValue task_list;
    this->global_handle.getParam("task_handler/valid_tasks", task_list);

    ROS_INFO("found %d tasks ", task_list.size());
    for (unsigned i = 0; i < task_list.size(); i++) 
    {
      if (task_list[i].getType() == XmlRpcValue::TypeString) 
      {
        std::string content = task_list[i];
        ROS_INFO("Found a task in task_list: %s", content.c_str());
        this->valid_tasks.push_back(content);
      }
    }
  } 
  catch(XmlRpcException problem) 
  {
    ROS_WARN("error in setting up Communication Manager. Make sure valid task list is specified on parameter server. [%s]", problem.getMessage().c_str());
  }
}

void CommunicationManager::processBasePoseCB(nav_msgs::Odometry odom)
{
  recent_pose = odom;
}

void CommunicationManager::run() 
{
  ros::Rate rate(DEFAULT_ROS_RATE);

  while(ros::ok()) 
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void CommunicationManager::process_incoming_announcements_cb(
    arc_msgs::WirelessAnnouncement announcement) 
{
  ROS_DEBUG("Processing incoming wireless announcement");
}

bool CommunicationManager::isTaskRequestValid(arc_msgs::TaskRequest msg) 
{
  bool result = true;

  if(msg.task_id<=0) 
  {
    result = false;
    ROS_WARN("Task request must have ID > 0. Task request %d ignored.", msg.task_id);
  }

  if(msg.created.sec<=0) 
  {
    result = false;
    ROS_WARN("Task creation time %d is invalid. Must be > 0", msg.created.sec);
  }

  if(msg.request_type==msg.TYPE_COMPLETION) 
  {
    if(std::find(this->valid_tasks.begin(), valid_tasks.end(), msg.task_name)==valid_tasks.end()) 
    {
      ROS_WARN("Task name %s is not a valid task. Task request %d ignored.", msg.task_name.c_str());
      result = false;
    }
  }

  return result;
}

void CommunicationManager::process_incoming_requests_cb(arc_msgs::WirelessRequest request) 
{
  ROS_DEBUG("Processing incoming wireless request");

  if(this->isTaskRequestValid(request.task)) 
  {
    arc_msgs::TaskRequest task = request.task;
    this->task_requests_pub.publish(task);
  }
}

void CommunicationManager::process_incoming_responses_cb(arc_msgs::WirelessResponse response) 
{
  ROS_DEBUG("Processing incoming wireless response");
  //TODO Check if request is meant for us

  // Process the different response types accordingly
  if (response.type == response.TASK_CONFIRMATION)
  {
    arc_msgs::TaskConfirmation confirmation;
    
    ROS_ASSERT(response.response.bools.back().name == "confirmation");
    ROS_ASSERT(response.response.ints.back().name == "task_id");

    confirmation.confirmation = response.response.bools.back().value;
    confirmation.task_id = response.response.ints.back().value; 
    task_confirmations_pub.publish(confirmation);
  }

}

//-------------------------------------
//
// OUTGOING MESSAGES
//
//-------------------------------------

void CommunicationManager::sendTaskAcknowledgement(arc_msgs::TaskAcknowledgement acknowledgement)
{
  ROS_INFO("Sending task %d acknowledgement", acknowledgement.task_id);
  arc_msgs::WirelessResponse wifi_response;
  prepareWirelessMessage(wifi_response);

  wifi_response.type = wifi_response.TASK_ACKNOWLEDGEMENT;
  dynamic_reconfigure::IntParameter task_id;

  // Populate acknowledgement data in the wifi message
  task_id.name = "task_id";
  task_id.value = acknowledgement.task_id;
  wifi_response.response.ints.push_back(task_id);

  outgoing_responses_pub.publish(wifi_response);
}

void CommunicationManager::sendTaskResponse(arc_msgs::TaskResponse response)
{
  ROS_INFO("Sending an outgoing task response");

  arc_msgs::WirelessResponse wifi_response;
  wifi_response.type = wifi_response.TASK_RESPONSE;

  prepareWirelessMessage(wifi_response);

  // Store our response info in the wifi message
  dynamic_reconfigure::IntParameter task_id, suitability;
  dynamic_reconfigure::BoolParameter accepted;
  
  task_id.name = "task_id";
  suitability.name = "suitability";
  accepted.name = "accepted";
  
  task_id.value = response.task_id;
  suitability.value = response.suitability;
  accepted.value = response.accepted;

  wifi_response.response.ints.push_back(task_id);
  wifi_response.response.ints.push_back(suitability);
  wifi_response.response.bools.push_back(accepted);

  outgoing_responses_pub.publish(wifi_response);
}

template<typename T>
void CommunicationManager::prepareWirelessMessage(T& msg)
{
  // Populate with information about the robot
  arc_msgs::BotInfoRequest req;
  bot_info_client.call(req);
  msg.info = req.response.info;

  msg.sender_location = recent_pose.pose.pose;
}


