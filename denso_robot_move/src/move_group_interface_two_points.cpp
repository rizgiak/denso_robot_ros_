// Code based on moveit_tutorial with several changes
// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp
// --------------------------------------------------

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

std::string printRobotPose(geometry_msgs::Pose pose)
{
  std::stringstream ss;
  ss << "[Pos] ";
  ss << "X:" << pose.position.x << "\t";
  ss << "Y:" << pose.position.y << "\t";
  ss << "Z:" << pose.position.z << "\t";
  ss << "[Or] ";
  ss << "W:" << pose.orientation.w << "\t";
  ss << "X:" << pose.orientation.x << "\t";
  ss << "Y:" << pose.orientation.y << "\t";
  ss << "Z:" << pose.orientation.z << "";
  return ss.str();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Set the planning group that will be used
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //Set Parameters
  move_group.setMaxVelocityScalingFactor(0.7);
  move_group.setMaxAccelerationScalingFactor(0.7);

  // Visualization
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = -0.075;
  visual_tools.publishText(text_pose, "ntlab", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("denso_robot_move", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("denso_robot_move", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  std::string planning_groups;
  ROS_INFO_NAMED("denso_robot_move", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  ROS_INFO("\n");

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  ROS_INFO_STREAM("[ROBOT POSE]  " + printRobotPose(current_pose.pose));

  move_group.setPoseReferenceFrame("base_link");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Set waypoints to display the way point in rviz
  std::vector<geometry_msgs::Pose> waypoints;

  waypoints.push_back(current_pose.pose);

  // Planning to a Pose goal 1
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.0738944;
  target_pose1.position.y = 0.185343;
  target_pose1.position.z = 0.180862;
  target_pose1.orientation.w = 0.0355113;
  target_pose1.orientation.x = 0.78108;
  target_pose1.orientation.y = -0.623277;
  target_pose1.orientation.z = -0.0134038;
  move_group.setPoseTarget(target_pose1);

  waypoints.push_back(target_pose1);

  ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

  // Set plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a pose goal
  move_group.move();

  //Show current pose of robot.
  current_pose = move_group.getCurrentPose();
  ROS_INFO_STREAM("[ROBOT POSE]  " + printRobotPose(current_pose.pose));

  waypoints.clear();
  waypoints.push_back(current_pose.pose);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  target_pose1.position.x = 0.0253302;
  target_pose1.position.y = 0.181333;
  target_pose1.position.z = 0.338335;
  target_pose1.orientation.w = 0.0355112;
  target_pose1.orientation.x = 0.78108;
  target_pose1.orientation.y = -0.623278;
  target_pose1.orientation.z = -0.013326;
  move_group.setPoseTarget(target_pose1);

  waypoints.push_back(target_pose1);

  ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 as trajectory line");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "pose2");
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a pose goal
  move_group.move();

  //Show current pose of robot.
  current_pose = move_group.getCurrentPose();
  ROS_INFO_STREAM("[ROBOT POSE]  " + printRobotPose(current_pose.pose));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  ros::shutdown();
  return 0;
}
