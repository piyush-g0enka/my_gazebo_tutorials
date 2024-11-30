/**
 * @file my_walker.hpp
 * @brief Header file of the walker node
 *
 * This file contains the Class and function declarations of the Robot, 
 * RobotState, ForwardMoveState and RotateMoveState classes. 
 *
 * @author Piyush Goenka
 * @date 2024
 */
 
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RobotState;

/**
 * @class Robot
 * @brief A ROS2 node for sending control commands to turtlebot burger
 * @details
 * The Robot node initializes the robot to move forward and provides methods to
 * change the robot's state, give velocity commands, check for obstacles, and
 * change the rotation direction.
 */
class Robot : public rclcpp::Node {
public:
  /**
   * @brief Constructor for initializing the Robot walker node.
   *
   * Sets up publishers, subscribers, and initializes the robot in
   * ForwardMoveState.
   */
  Robot();

  /**
   * @brief Changes the current state of the Robot.
   * @param new_robot_state Pointer to the new state
   *
   * Goes from the current state to the new state.
   */
  void change_robot_state(RobotState *new_robot_state);

  /**
   * @brief Publishes velocity commands to the robot.
   * @param linear_velocity Linear velocity in m/s.
   * @param angular_velocity Angular velocity in rad/s.
   */
  void give_velocity_command(double linear_velocity, double angular_velocity);

  /**
   * @brief Checks if the path in front does not have obstacle.
   * @param scan pointer to the laser scan data.
   * @return if obstacle is present then return false, else true
   */
  bool check_path(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

  /**
   * @brief Changes the rotation direction ( clockwise <--> anti-clockwise )
   */
  void change_rotation_direction();

  /**
   * @brief Gets the current rotation direction.
   * @return anti-clockwise (1) or clockwise (-1)
   */
  double get_rotation_direction() const { return rotation_direction_; }

  /**
   * @brief Creates a timer with specified period and callback.
   * @param period Duration between timer callbacks.
   * @param callback Function to be called when timer expires.
   * @return Shared pointer to the created timer.
   */
  rclcpp::TimerBase::SharedPtr
  create_timer(const std::chrono::duration<double> &period,
               std::function<void()> callback);

private:
  /**
   * @brief Callback function for processing laser scan messages.
   * @param scan Shared pointer to the received laser scan message.
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  RobotState *current_robot_state; // Pointer to current state object
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      velocity_publisher; // Velocity command publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      lidar_scan_subscriber;       // Laser scan subscriber
  double rotation_direction_; // Current rotation direction
  const double SAFE_DISTANCE = 1.0; // Minimum safe distance from obstacles in meters
};

/**
 * @class RobotState
 * @brief Abstract base class for Robot states.
 */
class RobotState {
public:
  /**
   * @brief Virtual destructor ensuring proper cleanup of derived classes.
   */
  virtual ~RobotState() = default;

  /**
   * @brief Pure virtual function to handle robot behavior in current state.
   * @param walker Pointer to the Robot object.
   * @param scan Shared pointer to laser scan data.
   */
  virtual void handle(Robot *walker,
                      const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

/**
 * @class ForwardMoveState
 * @brief Forward movement state (zero angular velocity)
 */
class ForwardMoveState : public RobotState {
public:
  /**
   * @brief Handles forward movement and obstacle detection.
   * @param walker Pointer to the Walker object.
   * @param scan Shared pointer to laser scan data.
   */
  void handle(Robot *walker,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

/**
 * @class RotationMoveState
 * @brief Rotation state ( angular velocity !=0)
 */
class RotateMoveState : public RobotState {
public:
  /**
   * @brief Constructor initializing rotation state parameters.
   */
  RotateMoveState() : initial_rotation_(true), rotation_timer_(nullptr) {}

  /**
   * @brief Handles rotation behavior and state transitions.
   * @param walker Pointer to the Robot object.
   * @param scan Shared pointer to laser scan data.
   */
  void handle(Robot *walker,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;

private:
  bool initial_rotation_; // Flag tracking initial rotation period
  rclcpp::TimerBase::SharedPtr
      rotation_timer_; // Timer for managing rotation duration
};
