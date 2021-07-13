#include "../include/TaskHandler.h"
#include "ros/ros.h"
#include "arc_msgs/TaskSuitability.h"
#include "std_msgs/String.h"
#include <XmlRpcException.h>

using namespace XmlRpc;

#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10

typedef arc_msgs::TaskRequest TaskGoal;

TaskHandler::TaskHandler() : 
  local_handle("task_handler"), 
  task_coordination_client("task_coordination")
{
//    ros::SubscribeOptions options = ros::SubscribeOptions::create<TaskGoal>("/string_topic", MAX_QUEUE_SIZE,&
//:
  this->task_request_sub = this->local_handle.subscribe("task_requests", MAX_QUEUE_SIZE, 
      &TaskHandler::task_handler_goal_cb, this);

  startupActionClients();
}

int TaskHandler::isAcceptableRequest(const TaskHandler::TaskGoal &goal) 
{
  int suitability{ 0 };
  ros::ServiceClient client;
  arc_msgs::TaskSuitability srv;

  //check if task with this name exists
  if(suitability_clients.count(goal.task_name)) 
  {
    suitability_clients.at(goal.task_name).call(srv);
    suitability = srv.response.suitability;
  }
  
  return suitability;
}

void TaskHandler::startupActionClients() 
{
  ROS_INFO_NAMED("TaskHandler", "Setting up action clients.");
  XmlRpcValue task_names;

  try 
  {
    ros::param::get("task_handler/valid_tasks", task_names);

    for(unsigned i=0; i< task_names.size(); i++) 
    {
      if(task_names[i].getType()==XmlRpcValue::TypeString) 
      {
        std::string content = task_names[i];
        ROS_INFO_NAMED("TaskHandler", "Found task named %s", content.c_str());
        std::string task_name = "task_" + content;

        // Setup the action client for the task
        ActionClient *client = new ActionClient(task_name, true);
        this->task_clients.insert({content, client});

        // Setup the suitability service client for the task
        ros::ServiceClient srv_client = 
          global_handle.serviceClient<arc_msgs::TaskSuitability>(
              task_name + "_server/suitability");
        suitability_clients.insert({content, srv_client});
      } 
      else 
      {
        //ROS_WARN_NAMED("TaskHandler", "Found non-string value in task_list. Value ignored.");
      }
    }
  } 
  catch (XmlRpcException &ex) 
  {
    ROS_WARN_NAMED("TaskHandler", "Problem setting up ActionClients for Task_handler %s", 
        ex.getMessage().c_str());
  }
}

int TaskHandler::getBacklogSize() 
{
  return this->task_backlog.size();
}

//------------------------------------
//
// Handle task requests recieved by the robot
//
//------------------------------------

void TaskHandler::processRequest(const TaskHandler::TaskGoal &goal) 
{
  int suitability = isAcceptableRequest(goal);
  ROS_INFO("Processing task with suitability %d.", suitability);

  if(!suitability && getBacklogSize() < max_backlog) 
  {
    rejectRequest(goal);
  } 
  else 
  {
    //call upon the task if it is acceptable
    ROS_DEBUG_NAMED("TaskHandler", "Accepting task request with id %d.", goal.task_id);
    acceptRequest(goal, suitability);
  }
}

void TaskHandler::acceptRequest(const TaskHandler::TaskGoal goal, int suitability) 
{
  //ROS_ASSERT(isAcceptableRequest(goal));
  
  // Our response to the request
  arc_msgs::TaskCoordinationGoal task_response;
  task_response.response.task_id = goal.task_id;
  task_response.response.accepted = true;
  task_response.response.suitability = suitability;
  task_coordination_client.sendGoal(task_response);

  bool confirmation_before_timeout = task_coordination_client.waitForResult(confirmation_timeout);

  if (confirmation_before_timeout)
  {
    //TODO test: If task is accepted, test to ensure it is in backlog immediatly after
    this->task_backlog.push_back(goal);
    //TODO Send acknowledgement to leader
  }
}

void TaskHandler::rejectRequest(const TaskHandler::TaskGoal goal) 
{
  ROS_INFO("Request rejected");

  // Populate then send our response
  arc_msgs::TaskCoordinationGoal task_response;
  task_response.response.task_id = goal.task_id;
  task_response.response.accepted = false;
  task_response.response.suitability = 0;   //suitability doesn't matter, we're rejecting it anyway

  task_coordination_client.sendGoal(task_response);
}

bool TaskHandler::isTaskActive(int task_id) 
{
  bool found = false;

  for(int pos=0; pos < this->active_tasks.size(); pos ++ ) 
  {
    if(this->active_tasks.at(pos).task_id==task_id) 
    {
      found = true;
    }
  }

  return found;
}

void TaskHandler::markTaskCompleted(int task_id) 
{
  TaskGoal task;
  int task_pos = 0;

  ROS_INFO_NAMED("TaskHandler", "Task [%d] has been completed.", task_id);
  for(int pos=0; pos< this->active_tasks.size(); pos++) 
  {
    if(this->active_tasks.at(pos).task_id==task_id) 
    {
      task = active_tasks.at(pos);
      task_pos = pos;
    }
  }

  this->active_tasks.erase(this->active_tasks.begin()+task_pos);
}

void TaskHandler::attemptToBeginNewTasks() 
{
  //if we aren't doing any work, add a task to active_task list
  if(this->active_tasks.empty()) 
  {
    ROS_INFO_NAMED("TaskHandler", "Currently [%d] active tasks, and [%d] tasks in queue.", 
        this->getNumActiveTasks(), this->getBacklogSize());
    if(this->getBacklogSize()>0) 
    {
      ROS_INFO("starting task!)");
      TaskGoal task = this->task_backlog.at(0);

      this->beginTask(task);
      //boost::thread_group t_group;
      //t_group.create_thread(boost::bind(&TaskHandler::beginTask, this, task));
    }
  } 
  else 
  {
  }
}

int TaskHandler::getNumActiveTasks() 
{
  return this->active_tasks.size();
}

void TaskHandler::beginTask(const TaskHandler::TaskGoal goal) 
{
  arc_msgs::ArcTaskGoal action_goal; //breaking down project into action
  action_goal.parameters = goal.parameters;
  action_goal.task_id = goal.task_id;
  ROS_INFO("starting new TASK WITH ID = %d", goal.task_id);
  boost::thread_group t_group;
  this->task_clients.at(goal.task_name)->sendGoal(action_goal, 
      boost::bind(&TaskHandler::task_handler_result_cb, this, _1, _2));
  int pos_of_task = 0;

  for(int pos=0; pos< this->task_backlog.size(); pos++) 
  {
    if(this->task_backlog.at(pos).task_id==goal.task_id) 
    {
      pos_of_task = pos;
    }
  }
  this->task_backlog.erase(this->task_backlog.begin()+pos_of_task);
  this->active_tasks.push_back(goal);
}

bool TaskHandler::isTaskInBacklog(int task_id) 
{
  bool found = false;

  for(int pos=0; pos < this->task_backlog.size(); pos ++ ) 
  {
    if(this->task_backlog.at(pos).task_id==task_id) 
    {
      ROS_INFO("Found task with id %d", task_backlog.at(pos).task_id);
      found = true;
    }
  }

  return found;
}


void TaskHandler::process() 
{//update tasks
  ros::Rate rate(DEFAULT_RATE);
  //timer periodically checks if we can perform a new task yet
  ros::Timer timer = global_handle.createTimer(ros::Duration(1), 
      &TaskHandler::attempt_to_begin_new_tasks_timer_cb, this, false);
  timer.start();

  //asynchronous spinner to handle task request callbacks
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok()) 
  {
    rate.sleep();
  }
  //ensure callbacks are called when task requests come in.
}

void TaskHandler::task_handler_goal_cb(const TaskHandler::TaskGoal &goal) 
{
  ROS_INFO_NAMED("TaskHandler","Received task request!");
  this->processRequest(goal);
}

void TaskHandler::attempt_to_begin_new_tasks_timer_cb(const ros::TimerEvent &event) 
{
  this->attemptToBeginNewTasks();
}

void TaskHandler::task_handler_result_cb(const actionlib::SimpleClientGoalState &state,
                                         const arc_msgs::ArcTaskResultConstPtr &result) 
{
  this->markTaskCompleted(result->task_id);
}
