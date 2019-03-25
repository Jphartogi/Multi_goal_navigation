#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
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
  void run(int status);
  int goal_count;
  bool goal_reached ,goal_sended;

  
  double real_start_time, real_end_time;

  int i , j;

  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal goal2;
  
  int check_status(int status);
  double get_start_time(double start_time);
  double get_end_time(double end_time);
  int goal_status;
  // std::map<std::string, Pose> goal_map_;
  

public:
  void resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
  
  Multigoal(ros::NodeHandle nh);
  ~Multigoal();
};

Multigoal::Multigoal(ros::NodeHandle nh)
{
  i = 0;
  j = 0;
  getGoals();
  sub = nh.subscribe("/move_base/status",1,&Multigoal::resultCallback,this);
  goal_count = 0;

}

Multigoal::~Multigoal()
{
}

void Multigoal::run(int status)
{
  
  if (status == 3 && goal_count == 0) { // this is the first stage 
    
    // still in the first stage but already in some position from previous action
    goal_reached = false;
    callActionServer(goal);
    goal_count = goal_count + 1;
    ROS_INFO("goal count mustinya 1 %i ",goal_count);
  }
  if(goal_count == 1 && status == 1)
  {
    // already succesfully sending the goal.
    goal_count = goal_count + 1;
     ROS_INFO("goal count mustinya 2 %i ",goal_count);
  }
  if (goal_count == 2 && status == 3) {
    // already send the goal and the first goal is reached
    callActionServer(goal2);
    goal_count = goal_count + 1;
     ROS_INFO("goal count mustinya 3 %i ",goal_count);
  }
  if (goal_count == 3 && status == 1) {
    // already successfully sending the second goal.
    goal_count = goal_count + 1;
    ROS_INFO("goal count mustinya 4 %i ",goal_count);
  }
  if (goal_count == 4 && status == 3) {
    ROS_INFO("all goal has reaced succesfully!");
    goal_count = goal_count + 1;
    ROS_INFO("goal count mustinya 5 %i ",goal_count);
    
  }
  else
  {
    //do nothing
  }
  
  
}

int Multigoal::check_status(int status)
{
  goal_status = status;
  // ROS_INFO("goal_status %i",goal_status);
  if (goal_status == 1) {
    // ROS_INFO("goal_sended ");
    goal_sended = true;
    goal_reached = false;
  }
  if (goal_status == 3)
  {
    // ROS_INFO("goal reached");
    goal_reached = true;
    goal_sended = false;
  }
  run(goal_status);
  return goal_status;
  // check the current status

}
double Multigoal::get_start_time(double start_time)
{
  
  if (start_time > 0 && i < 1 ) {
    real_start_time = start_time;
    std::cout << "real start time is :" << real_start_time << std::endl;
    return real_start_time;
    
  }
  
}
double Multigoal::get_end_time(double end_time){

  if (end_time > 0 && j < 1 ) {
    real_end_time = end_time;
    std::cout << "real end time is :" << real_end_time << std::endl;
    return real_end_time;
    
  }

}

void Multigoal::resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){
// check if goal is reached 
int goal_stat;
goal_stat = msg->status_list[0].status;
check_status(goal_stat);
double start_time , finish_time;

if (goal_count == 1)
		{
			// status is not clear, no goal is sended yet!
			start_time = msg->header.stamp.toSec(); // get the time from message
      get_start_time(start_time);
      i = i + 1;
      
      // getGoals();
     
		}
if ( goal_count == 5)
    {
      finish_time = msg->header.stamp.toSec(); // get the time from message
      get_end_time(finish_time);
      j = j + 1;
      double total_time = real_start_time - real_end_time;
      ROS_INFO("total time = %f",total_time);
      goal_count = goal_count + 1;
    }
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
  // after the goal has been obtained, now run the code to go to the destination
  
  }
  else
  {
    ROS_INFO("No param 'goals' found in an upward search");
  }

}

void Multigoal::setGoals(Pose final_pose,double goal_num)
{
  
  if (goal_num == 0) {
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = final_pose.frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = final_pose.x;
  goal.target_pose.pose.orientation.w = final_pose.theta;
  std::cout << "ini yang pertama bro " << final_pose.x << ", " << final_pose.y << ", " << final_pose.theta << ", " << final_pose.frame << std::endl;
  
  
  }
  if (goal_num == 1)
  {
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward
  goal2.target_pose.header.frame_id = final_pose.frame;
  goal2.target_pose.header.stamp = ros::Time::now();
  
  goal2.target_pose.pose.position.x = final_pose.x;
  goal2.target_pose.pose.orientation.w = final_pose.theta;
  
  std::cout << "ini yang kedua bro " << final_pose.x << ", " << final_pose.y << ", " << final_pose.theta << ", " << final_pose.frame << std::endl;
    
    
  }
  
}

void Multigoal::callActionServer(move_base_msgs::MoveBaseGoal goal)
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
   MoveBaseClient ac("/move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
  ROS_INFO("Waiting for the move_base action server to come up");
    
  }
  //we'll send a goal to the robot the goal we get from previous function
  ac.sendGoal(goal);
  ROS_INFO("Sending goal");
  
  
}

int main(int argc, char** argv){
    ros::init(argc,argv,"multi_goal_driver");
    ros::NodeHandle nh;
    Multigoal Multigoal(nh);  
    ros::Rate rate(5);
    ros::spin();

    return 0;

  //wait for the action server to come up

}



