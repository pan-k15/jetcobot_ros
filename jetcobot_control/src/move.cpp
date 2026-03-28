#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_plan_cpp");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface scene;
    moveit::planning_interface::MoveGroupInterface jetcobot("arm_group");

    // Planning parameters
    jetcobot.allowReplanning(true);
    jetcobot.setPlanningTime(50.0);
    jetcobot.setNumPlanningAttempts(20);

    jetcobot.setGoalPositionTolerance(0.01);
    jetcobot.setGoalOrientationTolerance(0.01);
    jetcobot.setGoalTolerance(0.01);

    jetcobot.setMaxVelocityScalingFactor(1.0);
    jetcobot.setMaxAccelerationScalingFactor(1.0);

    ROS_INFO("Reset pose");

    jetcobot.setNamedTarget("init");
    jetcobot.move();

    ros::Duration(0.5).sleep();

    ROS_INFO("Ready pose");

    std::vector<double> joints = {
        -0.07339833069193903,
        0.9798838167728141,
        -2.5519868536114148,
        1.57218173439937,
        0.07399901831847308,
        0.0011697565220774773};

    jetcobot.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan first_plan;
    jetcobot.plan(first_plan);
    jetcobot.execute(first_plan);

    ros::Duration(0.5).sleep();

    // Get current pose
    std::string end_effector_link = jetcobot.getEndEffectorLink();
    geometry_msgs::Pose start_pose =
        jetcobot.getCurrentPose(end_effector_link).pose;

    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(start_pose);

    for (int i = 0; i < 3; i++)
    {
        geometry_msgs::Pose wpose = start_pose;

        wpose.position.y -= 0.1;
        waypoints.push_back(wpose);

        wpose.position.y += 0.1;
        waypoints.push_back(wpose);
    }

    double fraction = 0.0;
    int maxtries = 100;
    int attempts = 0;

    moveit_msgs::RobotTrajectory trajectory;

    ROS_INFO("Path Planning in Cartesian Space");

    while (fraction < 1.0 && attempts < maxtries)
    {
        fraction = jetcobot.computeCartesianPath(
            waypoints,
            0.1,   // eef_step
            0.0,   // jump_threshold
            trajectory,
            true); // avoid collisions

        attempts++;

        if (attempts % 10 == 0)
        {
            ROS_INFO("Still trying after %d attempts...", attempts);
        }
    }

    if (fraction == 1.0)
    {
        ROS_INFO("Path computed successfully. Moving the jetcobot.");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        jetcobot.execute(plan);

        ROS_INFO("Path execution complete.");
    }
    else
    {
        ROS_WARN("Path planning failed with only %.2f success after %d attempts.",
                 fraction, maxtries);
    }

    ros::Duration(1.0).sleep();

    ros::shutdown();
    return 0;
}