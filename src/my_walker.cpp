// Copyright 2024 Piyush Goenka

/**
 * @file my_walker.cpp
 * @brief Header file of the walker node
 *
 * This file contains the Class and function definitions of the Robot,
 * RobotState, ForwardMoveState and RotateMoveState classes.
 * The class Robot is initialized as a ROS2 node.
 *
 * @author Piyush Goenka
 * @date 2024
 */

#include "walker/my_walker.hpp"

/**
 * @brief Constructs a new Robot object.
 *
 * Initializes the ROS2 node and sets up velocity publisher for robot control
 * and laser scan subscriber for obstacle detection.
 */
Robot::Robot() : Node("walker_node"), rotation_direction_(1.0) {
  auto qos_profile = rclcpp::QoS(10)
                         .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                         .history(rclcpp::HistoryPolicy::KeepLast);

  lidar_scan_subscriber =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos_profile,
          std::bind(&Robot::laser_callback, this, std::placeholders::_1));

  velocity_publisher =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  current_robot_state = new ForwardMoveState();
}

/**
 * @brief Callback function for processing laser scan messages.
 * @param scan Shared pointer to the received laser scan message.
 */
void Robot::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  current_robot_state->handle(this, scan);
}

/**
 * @brief Changes the current state of the Robot.
 * @param new_state Pointer to the new state object.
 *
 * Deletes the current state object and transitions to the new state.
 */
void Robot::change_robot_state(RobotState *new_state) {
  delete current_robot_state;
  current_robot_state = new_state;
}

/**
 * @brief Publishes velocity commands (/cmd_vel)
 * @param linear Linear velocity in m/s.
 * @param angular Angular velocity in rad/s.
 *
 * Creates and publishes a Twist message with the specified velocities.
 */
void Robot::give_velocity_command(double linear, double angular) {
  auto msg = geometry_msgs::msg::Twist();
  msg.angular.z = angular;
  msg.linear.x = linear;
  velocity_publisher->publish(msg);
}

/**
 * @brief Checks if the path ahead is clear of obstacles.
 * @param scan Shared pointer to the laser scan data.
 * @return true if path is clear, false if obstacle detected.
 *
 * Examines laser scan data in a 20-degree arc in front of the robot
 * (10 degrees on each side) for obstacles within SAFE_DISTANCE.
 */

bool Robot::check_path(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) const {

  const int left_start = 0;    // Front center
  const int left_end = 10;     // 10 degrees to the left
  const int right_start = 350;  // 10 degrees to the right
  const int right_end = 359;   // Back to front center

  // Check right side of front arc (350 to 359 degrees)
  for (int i = right_start; i <= right_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(rclcpp::get_logger("Walker"), "Obstacle detected!");
      return false;
    }
  }

  // Check left side of front arc (0 to 10 degrees)
  for (int i = left_start; i <= left_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(rclcpp::get_logger("Walker"), "Obstacle detected!");
      return false;
    }
  }

  return true;
}

/**
 * @brief Toggles the rotation direction of the robot.
 *
 * Multiplies the rotation_direction_ by -1.0 to switch between clockwise and
 * anti-clockwise rotation.
 */
void Robot::change_rotation_direction() { rotation_direction_ *= -1.0; }

/**
 * @brief Creates a timer with specified period and callback.
 * @param period Duration between timer callbacks.
 * @param callback Function to be called when timer expires.
 * @return Shared pointer to the created timer.
 */
rclcpp::TimerBase::SharedPtr
Robot::create_timer(const std::chrono::duration<double> &period,
                    std::function<void()> callback) {
  return this->create_wall_timer(period, callback);
}

/**
 * @brief Handles robot behavior in rotation state.
 * @param walker Pointer to the Walker object.
 * @param scan Shared pointer to laser scan data.
 *
 * Manages the robot's rotation behavior. Transitions to ForwardState when path
 * becomes clear.
 */
void RotateMoveState::handle(
    Robot *walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) {

  if (walker->check_path(scan)) {
    RCLCPP_INFO(walker->get_logger(), "Moving Forward!");
    walker->change_robot_state(new ForwardMoveState());
  } else {
    walker->give_velocity_command(0.0, 1.0 * walker->get_rotation_direction());
    RCLCPP_INFO(walker->get_logger(), "Rotating!");
  }
}

/**
 * @brief Handles robot behavior in forward state.
 * @param walker Pointer to the Walker object.
 * @param scan Shared pointer to laser scan data.
 *
 * Manages the robot's forward movement behavior. Moves forward when path
 * is clear and transitions to RotationState when obstacle is detected,
 * toggling rotation direction before transition.
 */
void ForwardMoveState::handle(
    Robot *walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->check_path(scan)) {
    walker->give_velocity_command(0.3, 0.0);
  } else {
    walker->change_rotation_direction();
    walker->change_robot_state(new RotateMoveState());
  }
}
