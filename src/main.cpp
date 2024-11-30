// Copyright 2024 Piyush Goenka

/**
 * @file main.cpp
 * @brief Main entry point for the Robot - walker node
 *
 * This file contains the main function for walker node
 *
 * @author Piyush Goenka
 * @date 2024
 */

#include "walker/my_walker.hpp"

/**
 * @brief Main function to start robot's control algorithm.
 *
 * Initializes an instance of the walker node,
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Exit code of the application (success if '0')
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto walker_node = std::make_shared<Robot>();
  rclcpp::spin(walker_node);
  rclcpp::shutdown();
  return 0;
}
