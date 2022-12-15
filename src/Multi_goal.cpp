#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <vector>
#include <mavros_msgs/SetMode.h>

enum status
{
  move_to_destination,
  attack_aim
};

struct Pose
{
  double x, y, theta, height;
  std::string frame;
};

class Multigoal
{
private:
  XmlRpc::XmlRpcValue goals;
  std::vector<move_base_msgs::MoveBaseGoal> goal;
  std::vector<int> current_task;
  ros::ServiceClient _set_mode_client;

  void callActionServer(move_base_msgs::MoveBaseGoal goal);
  void getGoals();
  void setGoals(Pose final_pose, int current_task);

  int goal_count;
  bool _verbose = true;

  mavros_msgs::SetMode _mode_cmd;
  ros::NodeHandle nh_;
  ros::Subscriber sub;

  int current_state = 0;

public:
  void
  resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
  Multigoal(ros::NodeHandle nh);
};

Multigoal::Multigoal(ros::NodeHandle nh)
{
  getGoals();
  sub = nh.subscribe("/move_base/status", 1, &Multigoal::resultCallback, this);
  _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  goal_count = 0;
}

void Multigoal::resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{

  int goal_stat;
  if (msg->status_list.empty())
  {
    goal_stat = 3;
  }
  else
  {
    goal_stat = msg->status_list[0].status;
  }

  // std::cout << "the current status:" << goal_stat << std::endl;
  static int cnt = 0;
  // std::cout << "goal status=" << goal_stat << " goal_count " << goal_count << std::endl;
  if (goal_stat == 3 && goal_count == 0)
  {
    std::cout << "the current cnt:" << cnt << " the size is:" << goal.size() << std::endl;
    if (cnt >= goal.size())
    {
      cnt = goal.size() - 1;
      _mode_cmd.request.custom_mode = "AUTO.LAND";
      _set_mode_client.call(_mode_cmd);
      ROS_INFO("AUTO LAND");
      _verbose = false;
    }
    if (_verbose)
    {
      callActionServer(goal[cnt]);
      goal_count = 1;
    }
  }

  if (goal_stat == 1 && goal_count == 1)
  {
    cnt = cnt + 1;
    // std::cout << "the cnt is :" << cnt << std::endl;
    goal_count = 0;
  }
}

void Multigoal::getGoals()
{

  std::string param_name;
  _verbose = true;

  if (nh_.searchParam("/multi_goal/multi_goal_driver/goals", param_name))
  {
    if (!nh_.hasParam("/multi_goal/multi_goal_driver/goals"))
    {
      ROS_ERROR("No stations on parameterserver");
    }

    // get Parameters from the yaml file
    nh_.getParam("/multi_goal/multi_goal_driver/goals", goals);
    for (size_t i = 0; i < goals.size(); i++)
    {
      XmlRpc::XmlRpcValue goal = goals[i];
      Pose final_pose;
      XmlRpc::XmlRpcValue poses = goal["poses"];
      std::string frame = goal["frame_id"];
      XmlRpc::XmlRpcValue pose_back = poses[poses.size() - 1];
      XmlRpc::XmlRpcValue current_tsk = goal["task"];
      final_pose.x = pose_back[0];
      final_pose.y = pose_back[1];
      final_pose.theta = pose_back[2];
      final_pose.frame = frame;
      setGoals(final_pose, current_tsk);
    }
    // after the goal has been obtained, now run the code to go to the destination
  }
  else
  {
    ROS_INFO("No param 'goals' found in an upward search");
  }
}

void Multigoal::setGoals(Pose final_pose, int current_task)
{
  static int cnt = 1;
  geometry_msgs::PoseStamped target_pose;
  move_base_msgs::MoveBaseGoal target_goal;

  // we'll send a goal to the robot to move 1 meter forward
  target_goal.target_pose.header.frame_id = final_pose.frame;
  target_goal.target_pose.header.stamp = ros::Time::now();

  target_goal.target_pose.pose.position.x = final_pose.x;
  target_goal.target_pose.pose.position.y = final_pose.y;
  target_goal.target_pose.pose.orientation.w = final_pose.theta;
  std::cout << "goal" << cnt << ":  " << final_pose.x << ", " << final_pose.y << ", " << final_pose.theta << ", " << final_pose.frame << std::endl;
  cnt++;

  goal.push_back(target_goal);
  current_task.push_back(current_task);
}

void Multigoal::callActionServer(move_base_msgs::MoveBaseGoal goal)
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac("/move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ac.sendGoal(goal);

  ROS_INFO("Sending goal");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_goal_driver");
  ros::NodeHandle nh;
  Multigoal Multigoal(nh);
  ros::Rate rate(5);
  ros::spin();

  return 0;
}
