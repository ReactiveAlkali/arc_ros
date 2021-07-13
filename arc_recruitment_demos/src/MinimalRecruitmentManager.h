/**
* CLASS: MinimalRecruitmentManager
* DATE: 13/04/17
* AUTHOR: Kyle Morris
* DESCRIPTION: A bare-bones recruitment manager that just listens for tasks and then performs them, nothing much more.
*/

#ifndef ARC_RECRUITMENT_DEMOS_MINIMALRECRUITMENTMANAGER_H
#define ARC_RECRUITMENT_DEMOS_MINIMALRECRUITMENTMANAGER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "arc_msgs/TaskCoordinationAction.h"
#include "arc_msgs/TaskRequest.h"
#include "arc_msgs/TaskConfirmation.h"

class MinimalRecruitmentManager 
{
private:
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle;

  /**
   * Subscribe to incoming task requests.
   */
  ros::Subscriber task_requests_sub;

  /**
   * Tasks we've accepted and are publishing to the task_handler to activate;
   */
  ros::Publisher task_request_pub;

  /**
   * Outgoing task responses publishing to the communication manager to send to team leader
   */
  ros::Publisher task_response_pub;

  /**
   * Subcribe to incoming task confirmations
   */
  ros::Subscriber task_confirmation_sub;

  /**
   * Coordinates the task assignment process between the task_handler and the team leader
   */
  actionlib::SimpleActionServer<arc_msgs::TaskCoordinationAction> task_coordination_server;

  /**
   * The result of the coordination
   */
  arc_msgs::TaskCoordinationResult task_confirmation;

  /**
   * ID of the task currently being coordinated
   */
  int task_id;

public:
  MinimalRecruitmentManager();

  /**
   * The main loop;
   */
  void process();

  void task_request_cb(const arc_msgs::TaskRequest &req);

  /**
   * Handles task responses coming from the task handler
   */
  void taskCoordinationCB();

  /**
   * Handles incoming task confirmations from communication manager
   */
  void taskConfirmationCB(const arc_msgs::TaskConfirmation& con);

  /**
   * Handles incoming preempt requests
   */
  void taskCoordPreemptCB();

};

#endif //ARC_RECRUITMENT_DEMOS_MINIMALRECRUITMENTMANAGER_H
