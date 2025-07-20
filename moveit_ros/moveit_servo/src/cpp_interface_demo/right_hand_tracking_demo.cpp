/*******************************************************************************
 *      Title     : hand_tracking_demo.cpp
 *      Project   : moveit_servo
 *      Created   : 06/22/2025
 *      Author    : Adapted from Adam Pettinger
 *
 * BSD 3-Clause License
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *******************************************************************************/

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // sensor_msgs/msg/JointState
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <mutex>
#include <optional>
#include <atomic>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.right_hand_tracking_demo");

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) { statusCB(msg); });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

// Helper to store latest pose from perception topic
class TargetPoseListener
{
public:
  TargetPoseListener(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_pose_ = *msg;
        received_ = true;
      }
    );
  }

  std::optional<geometry_msgs::msg::PoseStamped> getLatestPose()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_pose_;
  }

  bool hasReceived() const { return received_; }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
  std::atomic<bool> received_{false};
  mutable std::mutex mutex_;
};

// Subscribe to topic /joint_states for values of all joints
class JointStatesListener
{
public:
  JointStatesListener(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<sensor_msgs::msg::JointState>( // sensor_msgs/msg/JointState
      topic, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_joint_states_ = *msg;
        received_ = true;
      }
    );
  }

  std::optional<sensor_msgs::msg::JointState> getLatestJointStates()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_joint_states_;
  }

  bool hasReceived() const { return received_; }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_states_;
  std::atomic<bool> received_{false};
  mutable std::mutex mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("right_hand_tracking_demo");

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    exit(EXIT_FAILURE);
  }

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  // Wait for Planning Scene Monitor to setup
  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

  // Publisher for the servo pose command (could be used for visualization if needed)
  auto target_pose_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
  double rot_tol = 0.01;

  // Subscribe to hand pose perception
  TargetPoseListener perception_listener(node, "/pose_perception/right_end_effector_coords");

  // Subscribe to /joint_states
  JointStatesListener joint_states_listener(node, "/joint_states");

  // Wait for first content on the perception topic
  RCLCPP_INFO(LOGGER, "Waiting for a message on /pose_perception/right_end_effector_coords...");
  
  // Create executor in a separate thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  // Wait for the first message before starting the tracking thread
  rclcpp::Rate wait_rate(10);
  while (rclcpp::ok() && !perception_listener.hasReceived())
  {
    // executor.spin_some();
    wait_rate.sleep();
  }
  RCLCPP_INFO(LOGGER, "Received first pose from perception topic!");

  // Make sure the robot arm moves to the first received pose firstly
  // The user should keep the fist pose for a while before starting the tracking
  // Give the robot some time to move to the initial pose

  // Get the initial pose from perception
  auto received_init_pose = perception_listener.getLatestPose();
  RCLCPP_INFO(LOGGER, "Received initial target pose: "
    "x: %.3f, y: %.3f, z: %.3f, orientation: [%.3f, %.3f, %.3f, %.3f]",
    received_init_pose->pose.position.x,
    received_init_pose->pose.position.y,
    received_init_pose->pose.position.z,
    received_init_pose->pose.orientation.x, 
    received_init_pose->pose.orientation.y,
    received_init_pose->pose.orientation.z, 
    received_init_pose->pose.orientation.w
  );

  // Adjust the initial pose
  geometry_msgs::msg::PoseStamped target_init_pose;
  target_init_pose.header.frame_id = received_init_pose->header.frame_id;
  target_init_pose.header.stamp = node->now();
  target_init_pose.pose.position.x = received_init_pose->pose.position.x;
  target_init_pose.pose.position.y = 0.0;
  target_init_pose.pose.position.z = received_init_pose->pose.position.z;
  target_init_pose.pose.orientation = received_init_pose->pose.orientation;
  RCLCPP_INFO(LOGGER, "Target initial target pose: "
    "x: %.3f, y: %.3f, z: %.3f, orientation: [%.3f, %.3f, %.3f, %.3f]",
    target_init_pose.pose.position.x,
    target_init_pose.pose.position.y,
    target_init_pose.pose.position.z,
    target_init_pose.pose.orientation.x, 
    target_init_pose.pose.orientation.y,
    target_init_pose.pose.orientation.z, 
    target_init_pose.pose.orientation.w
  );

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
  tracker.resetTargetPose();

  // Publish initial pose
  target_pose_pub->publish(target_init_pose);

  // Run the pose tracking in a new thread
  std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
    moveit_servo::PoseTrackingStatusCode tracking_status =
        tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
    RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                   << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
  });

  // Main loop: whenever a new pose is received, send to servo
  rclcpp::WallRate thread_rate(200); // ori:50
  while (rclcpp::ok())
  {
    auto latest_pose = perception_listener.getLatestPose();
    geometry_msgs::msg::PoseStamped target_pose;

    // deal-arm right arm, i.e. rotated and translated
    target_pose.header.frame_id = latest_pose->header.frame_id;
    target_pose.header.stamp = node->now();
    target_pose.pose.position.x = latest_pose->pose.position.x;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = latest_pose->pose.position.z;
    target_pose.pose.orientation = latest_pose->pose.orientation;

    // RCLCPP_INFO(LOGGER, "Received new target pose: "
    //   "x: %.3f, y: %.3f, z: %.3f, orientation: [%.3f, %.3f, %.3f, %.3f]",
    //   target_pose.pose.position.x, 
    //   target_pose.pose.position.y,
    //   target_pose.pose.position.z,
    //   target_pose.pose.orientation.x, 
    //   target_pose.pose.orientation.y,
    //   target_pose.pose.orientation.z, 
    //   target_pose.pose.orientation.w
    // );

    // // show current joint states
    // auto latest_joint_states = joint_states_listener.getLatestJointStates();
    // if (latest_joint_states) {
    //   RCLCPP_INFO(LOGGER, "Current joint states: "
    //     "joint1: %.3f, joint2: %.3f, joint3: %.3f, joint4: %.3f, joint5: %.3f, joint6: %.3f, joint7: %.3f,",
    //     latest_joint_states->position[1], 
    //     latest_joint_states->position[2],
    //     latest_joint_states->position[4],
    //     latest_joint_states->position[5],
    //     latest_joint_states->position[6],
    //     latest_joint_states->position[7],
    //     latest_joint_states->position[8]
    //   );
    // }



    // Publish for visualization/feedback, optional
    target_pose_pub->publish(target_pose);

    thread_rate.sleep();
  };

  // Make sure the tracker is stopped and clean up
  move_to_pose_thread.join();

  // Kill executor thread before shutdown
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}