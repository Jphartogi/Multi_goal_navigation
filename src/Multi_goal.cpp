#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <string.h>
#include <map>
#include <nav_msgs/Odometry.h>

struct Pose
{
  double x, y, theta;
  std::string frame;
  
};

class Multigoal
{
private:
  void callActionServer(move_base_msgs::MoveBaseGoal goal);
  void getGoals();
  void setGoals(Pose final_pose,double goal_num);
  void resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);

  bool goal_reached;
  double start_time, end_time;
  ros::NodeHandle nh_;
  // std::map<std::string, Pose> goal_map_;
  ros::Subscriber sub;

public:
  Multigoal(ros::NodeHandle nh);
  ~Multigoal();
};

Multigoal::Multigoal(ros::NodeHandle nh)
{
  getGoals();
  sub = nh.subscribe("/docker_control/move_base_linear/status",1,&Multigoal::resultCallback,this);
  goal_reached = false;
  start_time = 0;
  
}

Multigoal::~Multigoal()
{
}


void Multigoal::resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){
// check if goal is reached 
std::cout << "statusnya itu berapa sih " << std::endl; 
if (msg->status_list.empty())
		{
			// status is not clear, no goal is sended yet!
			double time = msg->header.stamp.toSec(); // get the time from message
		
		}
		else{
		int status = msg->status_list[0].status;
    std::cout << "statusnya itu berapa sih " << status << std::endl; 

			
		}
goal_reached = true;
}

void Multigoal::getGoals()
{

std::string param_name;
  
  if (nh_.searchParam("/multi_goal/multi_goal_driver/goals", param_name))
  {
   XmlRpc::XmlRpcValue goals;
  if (!nh_.hasParam("/multi_goal/multi_goal_driver/goals"))
  {
    ROS_ERROR("No stations on parameterserver");
  }
  
  nh_.getParam("/multi_goal/multi_goal_driver/goals", goals);
  for (size_t i = 0 ; i < goals.size(); i++)
  {
    XmlRpc::XmlRpcValue goal = goals[i];
    Pose final_pose;
    XmlRpc::XmlRpcValue poses = goal["poses"];
    std::string frame = goal["frame_id"];
    XmlRpc::XmlRpcValue pose_back = poses[poses.size()-1];
    final_pose.x = pose_back[0];
    final_pose.y = pose_back[1];
    final_pose.theta = pose_back[2];
    final_pose.frame = frame;
    setGoals(final_pose,i);
  }

  }
  else
  {
    ROS_INFO("No param 'goals' found in an upward search");
  }

}



void Multigoal::setGoals(Pose final_pose,double goal_num)
{
  
  if (goal_num == 0) {
  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = final_pose.frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = final_pose.x;
  goal.target_pose.pose.orientation.w = final_pose.theta;
  std::cout << "ini yang pertama bro " << final_pose.x << ", " << final_pose.y << ", " << final_pose.theta << ", " << final_pose.frame << std::endl;
  callActionServer(goal);
  }
  else
  {
  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = final_pose.frame;
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = final_pose.x;
  goal.target_pose.pose.orientation.w = final_pose.theta;
  std::cout << "ini yang kedua bro " <<final_pose.x << ", " << final_pose.y << ", " << final_pose.theta << ", " << final_pose.frame << std::endl;
  if (goal_reached) {
    callActionServer(goal);
  }
  
  
  }
  
}

void Multigoal::callActionServer(move_base_msgs::MoveBaseGoal goal)
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
   MoveBaseClient ac("/docker_control/move_base_linear/", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    
  }
  
 
    std::cout << "start time is " << start_time << std::endl;
  //we'll send a goal to the robot the goal we get from previous function
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
    
    
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  // return 0;
}


int main(int argc, char** argv){
    ros::init(argc,argv,"multi_goal_driver");
    ros::NodeHandle nh;
    Multigoal Multigoal(nh);
    
  //wait for the action server to come up



}



