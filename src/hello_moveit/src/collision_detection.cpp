


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // <---- add this to the set of includes at the top


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "fr3_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup(
           "manipulator")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // First, get the current pose to understand where the end effector is
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current end effector position: x=%f, y=%f, z=%f", 
              current_pose.pose.position.x, 
              current_pose.pose.position.y, 
              current_pose.pose.position.z);
  RCLCPP_INFO(logger, "Current end effector orientation: x=%f, y=%f, z=%f, w=%f", 
              current_pose.pose.orientation.x, 
              current_pose.pose.orientation.y, 
              current_pose.pose.orientation.z, 
              current_pose.pose.orientation.w);

  // Set a target Pose with 180 degree rotation
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    
    // Quaternion for a 180-degree rotation around the Z-axis
    // For a 180-degree rotation around Z: x=0, y=0, z=1, w=0
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    
    // Position behind the box
    msg.position.x = 0.5;
    msg.position.y = -0.3;
    msg.position.z = 0.2;
    return msg;
  }();
  
  RCLCPP_INFO(logger, "Target end effector position: x=%f, y=%f, z=%f", 
              target_pose.position.x, 
              target_pose.position.y, 
              target_pose.position.z);
  RCLCPP_INFO(logger, "Target end effector orientation: x=%f, y=%f, z=%f, w=%f", 
              target_pose.orientation.x, 
              target_pose.orientation.y, 
              target_pose.orientation.z, 
              target_pose.orientation.w);
              
  move_group_interface.setPoseTarget(target_pose);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id =
    move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 0.2;
  primitive.dimensions[primitive.BOX_Z] = 0.2;

  // Define the pose of the box directly in front
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.3;  
  box_pose.position.y = 0.0;  
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Set some planning parameters to give more time
  move_group_interface.setPlanningTime(10.0); // 10 seconds planning time
  move_group_interface.setNumPlanningAttempts(5); // Try 5 different plans

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
    
    // If pose target planning fails, try an orientation target
    RCLCPP_INFO(logger, "Trying orientation target alone...");
    
    // Set only the orientation target
    move_group_interface.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    move_group_interface.setOrientationTarget(target_pose.orientation.x, target_pose.orientation.y, 
                                              target_pose.orientation.z, target_pose.orientation.w);
    
    draw_title("Re-planning with separate targets");
    moveit_visual_tools.trigger();
    
    auto const [success2, plan2] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();
    
    if (success2) {
      draw_trajectory_tool_path(plan2.trajectory_);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan2);
    } else {
      draw_title("All Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "All planning attempts failed!");
    }
  }
  
  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}