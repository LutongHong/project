#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    // initialise the ros node
    ros::init(argc, argv, "dvrk_planning");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // planning_group is the same as how it defined in config
    static const std::string PLANNING_GROUP = "psm1_arm";
    static const std::string PLANNING_GROUP_JAW = "psm1_gripper";

    // set up move_group class for control
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group_jaw(PLANNING_GROUP_JAW);

    // planning_scene_interface is to add and remove collision objects in virtual world scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    const robot_state::JointModelGroup *joint_model_group_jaw =
            move_group_jaw.getCurrentState()->getJointModelGroup(PLANNING_GROUP_JAW);

//    std::cout << move_group.getCurrentJointValues()[0] << std::endl;

    // MoveitVisualTools provides many capabilities for visualizing objects, robots and trajectories
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Generate marks in RViz
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // print out the reference frame
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    // print out the end-effector link
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // start the demo with the first path
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//    // Planning to a Cartesian-space goal
//    // p2p movement by setting the position of end-effector
//    geometry_msgs::Pose target_pose1;
//    target_pose1.orientation.x = 0.707;
//    target_pose1.orientation.y = -0.707;
//    target_pose1.orientation.z = 0.00;
//    target_pose1.orientation.w = 0.00;
//    target_pose1.position.x = -0.25;
//    target_pose1.position.y = 0.05;
//    target_pose1.position.z = 0.40;
//    move_group.setPoseTarget(target_pose1);
//
//    // the planner is called to compute the plan, but it is not executed yet
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_jaw;
//
//    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//
//    ROS_INFO("Visualizing plan 1 %s", success ? "" : "FAILED");
//
//    // Visualizing plans
//    ROS_INFO("Visualizing plan 1 as trajectory line");
//    visual_tools.publishAxisLabeled(target_pose1, "pose1");
//    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//    visual_tools.trigger();
//
//    /* Uncomment below line when working with a real robot */
//    move_group.move();
//
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Planning to a joint-space goal
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // get the current state of all the joints
    // current_state is a pointer
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // Planning to a joint-space goal
    moveit::core::RobotStatePtr current_state_jaw = move_group_jaw.getCurrentState();
    std::vector<double> joint_group_positions_jaw;
    current_state_jaw->copyJointGroupPositions(joint_model_group_jaw, joint_group_positions_jaw);

    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    move_group.move();
    move_group_jaw.setJointValueTarget(joint_group_positions_jaw);
    move_group_jaw.plan(my_plan_jaw);
    move_group_jaw.move();

    visual_tools.prompt("Press 'home' again to start demo");


    // Modify the joint values in joint space
    joint_group_positions[0] +=  0.1;  // radians
    joint_group_positions[1] +=  0.1;  // radians
    joint_group_positions[2] +=  0.1;  // radians
    joint_group_positions[3] +=  0.1;  // radians
    joint_group_positions[4] +=  0.1;  // radians
    joint_group_positions[5] +=  0.1;  // radians
    move_group.setJointValueTarget(joint_group_positions);



    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    joint_group_positions_jaw[0] = 0.5;
    move_group_jaw.setJointValueTarget(joint_group_positions_jaw);
    move_group_jaw.plan(my_plan_jaw);
    move_group_jaw.move();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//    // Planning with Path Constraints
//    moveit_msgs::OrientationConstraint ocm;
//    ocm.link_name = "PSM1_tool_wrist_sca_link";
//    ocm.header.frame_id = "PSM1_psm_base_link";
//    ocm.orientation.x = 0.0;
//    ocm.orientation.y = 0.707;
//    ocm.orientation.z = 0.0;
//    ocm.orientation.w = 0.707;
//    ocm.absolute_x_axis_tolerance = 0.1;
//    ocm.absolute_y_axis_tolerance = 0.1;
//    ocm.absolute_z_axis_tolerance = 0.1;
//    ocm.weight = 1.0;
//
//    // set the path constraint
//    moveit_msgs::Constraints dvrk_constraints;
//    dvrk_constraints.orientation_constraints.push_back(ocm);
//    move_group.setPathConstraints(dvrk_constraints);
//
//    // reset start state if needed
//    robot_state::RobotState start_state(*move_group.getCurrentState());
//    geometry_msgs::Pose start_pose2;
//    start_pose2.orientation.x = 0.707;
//    start_pose2.orientation.y = -0.707;
//    start_pose2.orientation.z = 0.0;
//    start_pose2.orientation.w = 0.0;
//    start_pose2.position.x = -0.25;
//    start_pose2.position.y = -0.05;
//    start_pose2.position.z = 0.40;
//    start_state.setFromIK(joint_model_group, start_pose2);
//    move_group.setStartState(start_state);
//
//    move_group.setPoseTarget(target_pose1);
//
//    // give time for planning with constraints as it can be slow
//    move_group.setPlanningTime(10.0);
//
//    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO("Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
//
//    // Visualize the plan in RViz
//    visual_tools.deleteAllMarkers();
//    visual_tools.publishAxisLabeled(start_pose2, "start");
//    visual_tools.publishAxisLabeled(target_pose1, "goal");
//    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//    visual_tools.trigger();
//    visual_tools.prompt("next step");
//
//    // clear the path constraint
//    move_group.clearPathConstraints();
//
//    // clear the start state
//    move_group.setStartStateToCurrentState();
//
//    // plan for Cartesian paths (waypoints)
//    geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
//
//    std::vector<geometry_msgs::Pose> waypoints;
//    waypoints.push_back(target_pose3);
//
//    target_pose3.position.z -= 0.2;
//    waypoints.push_back(target_pose3);
//
//    target_pose3.position.y -= 0.05;
//    waypoints.push_back(target_pose3);
//
//    target_pose3.position.x -= 0.05;
//    waypoints.push_back(target_pose3);
//
//    target_pose3.position.y += 0.1;
//    waypoints.push_back(target_pose3);
//
//    target_pose3.position.x += 0.1;
//    waypoints.push_back(target_pose3);
//
//    target_pose3.position.y -= 0.1;
//    waypoints.push_back(target_pose3);
//
//    target_pose3.position.x -= 0.05;
//    waypoints.push_back(target_pose3);
//
//
//    // set the speed smaller as Cartesian motions are frequently needed to be slower
//    move_group.setMaxVelocityScalingFactor(0.1);
//
//    // the Cartesian path is interpolated at a resolution of 1 cm (eef_step)
//    // the jump threshold is set to be 0.0 (disabled) change when operating real hardware
//    moveit_msgs::RobotTrajectory trajectory;
//    const double jump_threshold = 0.0;
//    const double eef_step = 0.01;
//    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//    ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
//
//    // Visualize the plan in RViz
//    visual_tools.deleteAllMarkers();
//    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//    visual_tools.trigger();
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//
//
//    // plan for Cartesian paths (waypoints)
//    geometry_msgs::Pose target_pose4 = move_group.getCurrentPose().pose;
//
//    std::vector<geometry_msgs::Pose> waypoints2;
//    waypoints2.push_back(target_pose4);
//
//    target_pose4.position.z -= 0.2;
//    waypoints2.push_back(target_pose4);
//
//    for (int i = 0; i < 50; i++){
//        target_pose4.position.x = -0.25 + 0.05 * std::cos(i * (2 * M_PI) / 50);
//        target_pose4.position.y = 0.05 * std::sin(i * (2 * M_PI) / 50);
//        waypoints2.push_back(target_pose4);
//    }
//
//    // set the speed smaller as Cartesian motions are frequently needed to be slower
//    move_group.setMaxVelocityScalingFactor(0.1);
//
//    // the Cartesian path is interpolated at a resolution of 1 cm (eef_step)
//    // the jump threshold is set to be 0.0 (disabled) change when operating real hardware
//    moveit_msgs::RobotTrajectory trajectory2;
//    double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
//    ROS_INFO("Visualizing plan 5 (Cartesian path) (%.2f%% acheived)", fraction2 * 100.0);
//
//    // Visualize the plan in RViz
//    visual_tools.deleteAllMarkers();
//    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishPath(waypoints2, rvt::LIME_GREEN, rvt::SMALL);
//    visual_tools.trigger();
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//
//    // Adding/Removing Objects
//    moveit_msgs::CollisionObject collision_object;
//    collision_object.header.frame_id = move_group.getPlanningFrame();
//
//    // The id of the object is used to identify it.
//    collision_object.id = "box1";
//
//    // Define a box to add to the world.
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = 0.5;
//    primitive.dimensions[1] = 0.2;
//    primitive.dimensions[2] = 0.35;
//
//    // Define a pose for the box (specified relative to frame_id)
//    geometry_msgs::Pose box_pose;
//    box_pose.orientation.w = 1.0;
//    box_pose.position.x = -0.25;
//    box_pose.position.y = 0.0;
//    box_pose.position.z = 0.175;
//
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(box_pose);
//    collision_object.operation = collision_object.ADD;
//
//    std::vector<moveit_msgs::CollisionObject> collision_objects;
//    collision_objects.push_back(collision_object);
//
//    // Now, let's add the collision object into the world
//    ROS_INFO_NAMED("tutorial", "Add an object into the world");
//    planning_scene_interface.addCollisionObjects(collision_objects);

//    // Show text in RViz of status
//    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//    visual_tools.trigger();
//
//    // Wait for MoveGroup to recieve and process the collision object message
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

//    // Now when we plan a trajectory it will avoid the obstacle
//    move_group.setStartState(*move_group.getCurrentState());
//    geometry_msgs::Pose another_pose;
//    another_pose.orientation.w = 1.0;
//    another_pose.position.x = 0.4;
//    another_pose.position.y = -0.4;
//    another_pose.position.z = 0.9;
//    move_group.setPoseTarget(another_pose);
//
//    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//
//    // Visualize the plan in RViz
//    visual_tools.deleteAllMarkers();
//    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//    visual_tools.trigger();
//    visual_tools.prompt("next step");
//
//    // Now, let's attach the collision object to the robot.
//    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
//    move_group.attachObject(collision_object.id);
//
//    // Show text in RViz of status
//    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
//    visual_tools.trigger();
//
//    /* Wait for MoveGroup to recieve and process the attached collision object message */
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
//                        "robot");
//
//    // Now, let's detach the collision object from the robot.
//    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
//    move_group.detachObject(collision_object.id);
//
//    // Show text in RViz of status
//    visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
//    visual_tools.trigger();
//
//    /* Wait for MoveGroup to recieve and process the attached collision object message */
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
//                        "robot");
//
//    // Now, let's remove the collision object from the world.
//    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
//    std::vector<std::string> object_ids;
//    object_ids.push_back(collision_object.id);
//    planning_scene_interface.removeCollisionObjects(object_ids);
//
//    // Show text in RViz of status
//    visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
//    visual_tools.trigger();
//
//    /* Wait for MoveGroup to recieve and process the attached collision object message */
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
//
//    // END_TUTORIAL


  ros::shutdown();
  return 0;
}
