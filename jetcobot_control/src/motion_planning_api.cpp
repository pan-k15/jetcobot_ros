#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  const std::string node_name = "jetcobot_motion_planning";
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // JETCOBOT SPECIFIC CONFIGURATION
  const std::string PLANNING_GROUP = "arm_group"; 
  const std::string BASE_LINK = "base_link";      
  const std::string EE_LINK = "6_Link";            

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues();

  // Load Planner Plugin
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    planner_plugin_name = "ompl_interface/OMPLPlanner"; // Default fallback

  try {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    planner_instance->initialize(robot_model, node_handle.getNamespace());
  } catch (pluginlib::PluginlibException& ex) {
    ROS_FATAL_STREAM("Exception while loading planner: " << ex.what());
    return 1;
  }

  // Visualization Setup
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_LINK);
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.loadRemoteControl();

  visual_tools.prompt("Press 'next' in RViz to plan to a Pose Goal");

  // 1. Pose Goal
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = BASE_LINK;
  // Adjusted coordinates for a small arm like JetCobot
  pose.pose.position.x = 0.15;
  pose.pose.position.y = 0.10;
  pose.pose.position.z = 0.20;
  pose.pose.orientation.w = 1.0;

  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(EE_LINK, pose, tolerance_pose, tolerance_angle);
  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan");
  } else {
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);
    visual_tools.publishTrajectoryLine(response.trajectory, joint_model_group);
    visual_tools.trigger();
  }

  visual_tools.prompt("Press 'next' to plan to a Joint Space Goal");

  // 2. Joint Space Goal
  moveit::core::RobotState goal_state(robot_model);
  // 6 joints for JetCobot (radians)
  std::vector<double> joint_values = { 0.5, -0.5, 0.5, 0.0, 0.5, 0.0 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);

  if (res.error_code_.val == res.error_code_.SUCCESS) {
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);
    visual_tools.publishTrajectoryLine(response.trajectory, joint_model_group);
    visual_tools.trigger();
  }

  visual_tools.prompt("Demo Finished. Press 'next' to exit.");
  return 0;
}