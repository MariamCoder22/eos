/**
 * @file eos_ros_node.cpp
 * @brief Main ROS2 node for Eos Robotics OS
 * 
 * This node integrates the Rust Eos core with ROS2, providing:
 * - Sensor data processing from ROS topics
 * - Neural network inference for decision making
 * - Navigation command publishing
 * - System status monitoring
 */

#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Forward declarations for our C++ classes that will interface with Rust
class NeuralBridge;
class NavigationController;

/**
 * @brief Main Eos ROS2 node class
 * 
 * Handles all ROS2 communication and coordinates between neural processing
 * and navigation control systems.
 */
class EosRosNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Eos Ros Node object
   */
  EosRosNode() : Node("eos_ros_node")
  {
    // Declare parameters with descriptions
    this->declare_parameter("neural_update_rate", 10.0);
    this->declare_parameter("navigation_update_rate", 15.0);
    this->declare_parameter("safety_distance", 0.5);
    this->declare_parameter("max_velocity", 0.5);
    this->declare_parameter("neural_model_path", "models/default_snn.json");
    
    // Get parameter values
    double neural_rate = this->get_parameter("neural_update_rate").as_double();
    double nav_rate = this->get_parameter("navigation_update_rate").as_double();
    safety_distance_ = this->get_parameter("safety_distance").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    std::string model_path = this->get_parameter("neural_model_path").as_string();

    RCLCPP_INFO(this->get_logger(), 
                "Eos ROS Node starting with neural rate: %.1fHz, navigation rate: %.1fHz", 
                neural_rate, nav_rate);

    // Initialize components
    initialize_components(model_path);
    
    // Create publishers
    initialize_publishers();
    
    // Create subscribers  
    initialize_subscribers();
    
    // Create timers for periodic processing
    initialize_timers(neural_rate, nav_rate);
    
    // Create services for external control
    initialize_services();

    RCLCPP_INFO(this->get_logger(), "Eos ROS Node initialized successfully");
  }

  /**
   * @brief Destroy the Eos Ros Node object
   */
  ~EosRosNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down Eos ROS Node");
    // Cleanup would happen here
  }

private:
  // ROS2 Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  
  // ROS2 Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  
  // Timers for periodic processing
  rclcpp::TimerBase::SharedPtr neural_timer_;
  rclcpp::TimerBase::SharedPtr navigation_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  
  // Component interfaces (these would interface with Rust code)
  std::unique_ptr<NeuralBridge> neural_bridge_;
  std::unique_ptr<NavigationController> navigation_controller_;
  
  // Current sensor data
  sensor_msgs::msg::LaserScan::SharedPtr current_laser_scan_;
  sensor_msgs::msg::Imu::SharedPtr current_imu_;
  nav_msgs::msg::Odometry::SharedPtr current_odometry_;
  
  // Parameters
  double safety_distance_;
  double max_velocity_;
  bool is_operational_ = false;

  /**
   * @brief Initialize neural and navigation components
   */
  void initialize_components(const std::string& model_path)
  {
    try {
      // These would create interfaces to the Rust code
      // neural_bridge_ = std::make_unique<NeuralBridge>(model_path);
      // navigation_controller_ = std::make_unique<NavigationController>();
      
      RCLCPP_INFO(this->get_logger(), "Components initialized successfully");
      is_operational_ = true;
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize components: %s", e.what());
      is_operational_ = false;
    }
  }

  /**
   * @brief Initialize ROS2 publishers
   */
  void initialize_publishers()
  {
    // Command velocity publisher for robot control
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
    
    // Status publisher for system monitoring
    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/eos/status", 10);
    
    // Goal publisher for navigation (optional)
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/eos/goal", 10);

    RCLCPP_INFO(this->get_logger(), "Publishers initialized");
  }

  /**
   * @brief Initialize ROS2 subscribers
   */
  void initialize_subscribers()
  {
    // Laser scan subscriber for obstacle detection
    laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->laser_callback(msg);
      });
    
    // IMU subscriber for orientation and acceleration
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->imu_callback(msg);
      });
    
    // Odometry subscriber for position tracking
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odom_callback(msg);
      });
    
    // Goal subscriber for receiving navigation goals
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/eos/set_goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->goal_callback(msg);
      });

    RCLCPP_INFO(this->get_logger(), "Subscribers initialized");
  }

  /**
   * @brief Initialize timers for periodic processing
   */
  void initialize_timers(double neural_rate, double nav_rate)
  {
    // Neural processing timer
    auto neural_interval = std::chrono::duration<double>(1.0 / neural_rate);
    neural_timer_ = this->create_wall_timer(
      neural_interval,
      [this]() { this->neural_processing_callback(); });
    
    // Navigation control timer
    auto nav_interval = std::chrono::duration<double>(1.0 / nav_rate);
    navigation_timer_ = this->create_wall_timer(
      nav_interval,
      [this]() { this->navigation_control_callback(); });
    
    // Status publishing timer (1Hz)
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { this->status_publishing_callback(); });

    RCLCPP_INFO(this->get_logger(), "Timers initialized");
  }

  /**
   * @brief Initialize ROS2 services for external control
   */
  void initialize_services()
  {
    // Services would be implemented here for:
    // - Starting/stopping navigation
    // - Loading new neural models
    // - Emergency stop
    // - System configuration
    
    RCLCPP_INFO(this->get_logger(), "Services initialized");
  }

  // =========================================================================
  // Callback Methods
  // =========================================================================

  /**
   * @brief Callback for laser scan data
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    current_laser_scan_ = msg;
    
    // Log first and last range for debugging
    if (!msg->ranges.empty()) {
      RCLCPP_DEBUG(this->get_logger(), 
                  "Laser scan: %zu points, first: %.2f, last: %.2f",
                  msg->ranges.size(), msg->ranges.front(), msg->ranges.back());
    }
  }

  /**
   * @brief Callback for IMU data
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    current_imu_ = msg;
    
    // Calculate magnitude of linear acceleration for basic activity monitoring
    double accel_magnitude = std::sqrt(
      msg->linear_acceleration.x * msg->linear_acceleration.x +
      msg->linear_acceleration.y * msg->linear_acceleration.y +
      msg->linear_acceleration.z * msg->linear_acceleration.z);
    
    RCLCPP_DEBUG(this->get_logger(), "IMU acceleration magnitude: %.2f", accel_magnitude);
  }

  /**
   * @brief Callback for odometry data
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odometry_ = msg;
    
    // Extract position for logging
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    RCLCPP_DEBUG(this->get_logger(), "Odometry position: (%.2f, %.2f)", x, y);
  }

  /**
   * @brief Callback for navigation goals
   */
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), 
                "Received new navigation goal: (%.2f, %.2f, %.2f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // This would be passed to the navigation controller
    // navigation_controller_->set_goal(msg);
  }

  /**
   * @brief Timer callback for neural network processing
   */
  void neural_processing_callback()
  {
    if (!is_operational_ || !current_laser_scan_ || !current_imu_) {
      return;
    }

    try {
      // Process sensor data through neural network
      // auto neural_output = neural_bridge_->process(
      //   current_laser_scan_, current_imu_, current_odometry_);
      
      // For now, simulate neural output
      std::vector<float> neural_output = {0.8f, 0.2f, 0.1f}; // Example output
      
      RCLCPP_DEBUG(this->get_logger(), 
                  "Neural processing completed, output size: %zu", 
                  neural_output.size());
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Neural processing failed: %s", e.what());
    }
  }

  /**
   * @brief Timer callback for navigation control
   */
  void navigation_control_callback()
  {
    if (!is_operational_) {
      return;
    }

    try {
      // Generate navigation commands based on current state
      // auto command = navigation_controller_->compute_command();
      
      // For now, publish a simple stop command
      auto stop_command = geometry_msgs::msg::Twist();
      stop_command.linear.x = 0.0;
      stop_command.angular.z = 0.0;
      
      cmd_vel_publisher_->publish(stop_command);
      
      RCLCPP_DEBUG(this->get_logger(), "Navigation control cycle completed");
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Navigation control failed: %s", e.what());
      
      // Emergency stop on failure
      auto emergency_stop = geometry_msgs::msg::Twist();
      emergency_stop.linear.x = 0.0;
      emergency_stop.angular.z = 0.0;
      cmd_vel_publisher_->publish(emergency_stop);
    }
  }

  /**
   * @brief Timer callback for status publishing
   */
  void status_publishing_callback()
  {
    auto status_msg = std_msgs::msg::String();
    
    if (is_operational_) {
      status_msg.data = "Eos OS: OPERATIONAL - Neural and navigation systems active";
    } else {
      status_msg.data = "Eos OS: DEGRADED - System initialization incomplete";
    }
    
    status_publisher_->publish(status_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Status published: %s", status_msg.data.c_str());
  }
};

/**
 * @brief Main function for Eos ROS2 node
 */
int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  try {
    // Create and spin the node
    auto node = std::make_shared<EosRosNode>();
    RCLCPP_INFO(node->get_logger(), "Eos ROS Node started successfully");
    
    // Keep the node running
    rclcpp::spin(node);
    
    // Clean shutdown
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("eos_ros_node"), 
                 "Fatal error in Eos ROS Node: %s", e.what());
    return 1;
  }
}