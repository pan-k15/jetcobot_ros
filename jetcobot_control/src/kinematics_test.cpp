#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jetcobot_kinematics_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Load the robot model from the parameter server
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  
  if (!kinematic_model) {
    ROS_ERROR("Could not load robot model! Is robot_description on the parameter server?");
    return 1;
  }

  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Prepare the RobotState
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  
  // NOTE: Yahboom/MyCobot usually uses "arm_group"
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm_group");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // 1. Get Joint Values
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // 2. Joint Limits Test
  // Setting Joint 0 to 5.0 radians (likely exceeds JetCobot limits)
  joint_values[0] = 5.0; 
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("State valid before enforcing bounds? " << (kinematic_state->satisfiesBounds() ? "Yes" : "No"));

  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("State valid after enforcing bounds? " << (kinematic_state->satisfiesBounds() ? "Yes" : "No"));

  // 3. Forward Kinematics (FK)
  // Target "link6" which is the typical end-effector link for JetCobot
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("6_Link");

  ROS_INFO_STREAM("End-Effector Translation: \n" << end_effector_state.translation() << "\n");

  // 4. Inverse Kinematics (IK)
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  if (found_ik)
  {
    ROS_INFO("IK solution found!");
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("IK Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // 5. Get the Jacobian
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

  ros::shutdown();
  return 0;
}