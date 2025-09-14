//! ROS 2 interface for Eos OS
//!
//! This module handles all communication with ROS 2, including:
//! - Publishing commands to actuators
//! - Subscribing to sensor data
//! - Managing ROS nodes and topics

mod publisher;
mod subscriber;

use r2r::{Context, Node, QosProfile};
use std::sync::{Arc, Mutex};
use std::time::Duration;

pub use publisher::*;
pub use subscriber::*;

/// ROS 2 interface manager
pub struct RosInterface {
    node: Arc<Node>,
    context: Context,
    publishers: RosPublishers,
    subscribers: RosSubscribers,
    is_initialized: bool,
}

/// Collection of all ROS publishers
pub struct RosPublishers {
    /// Command velocity publisher
    pub cmd_vel: Publisher<r2r::geometry_msgs::msg::Twist>,
    /// Status publisher
    pub status: Publisher<r2r::std_msgs::msg::String>,
    /// Neural output publisher
    pub neural_output: Publisher<r2r::std_msgs::msg::Float32MultiArray>,
}

/// Collection of all ROS subscribers
pub struct RosSubscribers {
    /// Laser scan subscriber
    pub laser_scan: Subscriber<r2r::sensor_msgs::msg::LaserScan>,
    /// IMU data subscriber
    pub imu: Subscriber<r2r::sensor_msgs::msg::Imu>,
    /// Odometry subscriber
    pub odom: Subscriber<r2r::nav_msgs::msg::Odometry>,
}

/// ROS interface status
#[derive(Debug, Clone)]
pub struct RosStatus {
    /// Node connectivity status
    pub connected: bool,
    /// Number of active publishers
    pub publishers_count: usize,
    /// Number of active subscribers
    pub subscribers_count: usize,
    /// Last message received time
    pub last_message_time: Option<std::time::SystemTime>,
}

impl RosInterface {
    /// Create a new ROS interface
    pub fn new(config: &super::RosConfig) -> Result<Self, RosError> {
        let context = Context::create()?;
        let node = Node::create(&context, &config.node_name, "")?;
        
        // Create QoS profile
        let qos = QosProfile::default()
            .depth(config.qos_depth)
            .reliability(r2r::QosReliabilityPolicy::BestEffort);
        
        // Initialize publishers and subscribers
        let publishers = RosPublishers::new(&node, &qos)?;
        let subscribers = RosSubscribers::new(&node, &qos)?;
        
        Ok(RosInterface {
            node: Arc::new(node),
            context,
            publishers,
            subscribers,
            is_initialized: false,
        })
    }
    
    /// Initialize the ROS interface
    pub fn initialize(&mut self) -> Result<(), RosError> {
        log::info!("Initializing ROS interface...");
        
        // Spawn a thread to handle ROS spinning
        let node_clone = self.node.clone();
        std::thread::spawn(move || {
            let mut executor = r2r::Executor::new();
            executor.add_node(node_clone).unwrap();
            
            loop {
                executor.spin_once(Duration::from_millis(100));
            }
        });
        
        self.is_initialized = true;
        log::info!("ROS interface initialized successfully");
        
        Ok(())
    }
    
    /// Get sensor data from ROS subscribers
    pub fn get_sensor_data(&self) -> Result<SensorData, RosError> {
        if !self.is_initialized {
            return Err(RosError::NotInitialized);
        }
        
        let laser_scan = self.subscribers.laser_scan.get_latest()?;
        let imu_data = self.subscribers.imu.get_latest()?;
        let odom_data = self.subscribers.odom.get_latest()?;
        
        Ok(SensorData {
            laser_scan,
            imu_data,
            odom_data,
        })
    }
    
    /// Publish a command to ROS
    pub fn publish_command(&self, command: &MotionCommand) -> Result<(), RosError> {
        if !self.is_initialized {
            return Err(RosError::NotInitialized);
        }
        
        // Convert internal command to ROS message
        let twist_msg = command.to_ros_message();
        
        // Publish the command
        self.publishers.cmd_vel.publish(&twist_msg)?;
        
        // Also publish status update
        let status_msg = r2r::std_msgs::msg::String {
            data: format!("Command published: {:?}", command),
        };
        self.publishers.status.publish(&status_msg)?;
        
        Ok(())
    }
    
    /// Publish neural network output
    pub fn publish_neural_output(&self, output: &[f32]) -> Result<(), RosError> {
        if !self.is_initialized {
            return Err(RosError::NotInitialized);
        }
        
        let array_msg = r2r::std_msgs::msg::Float32MultiArray {
            layout: r2r::std_msgs::msg::MultiArrayLayout {
                dim: vec![r2r::std_msgs::msg::MultiArrayDimension {
                    label: "output".to_string(),
                    size: output.len() as u32,
                    stride: output.len() as u32,
                }],
                data_offset: 0,
            },
            data: output.to_vec(),
        };
        
        self.publishers.neural_output.publish(&array_msg)?;
        
        Ok(())
    }
    
    /// Get current ROS status
    pub fn get_status(&self) -> RosStatus {
        RosStatus {
            connected: self.is_initialized,
            publishers_count: 3, // Fixed count for now
            subscribers_count: 3, // Fixed count for now
            last_message_time: self.subscribers.laser_scan.get_last_message_time(),
        }
    }
    
    /// Shutdown the ROS interface
    pub fn shutdown(&mut self) -> Result<(), RosError> {
        log::info!("Shutting down ROS interface...");
        
        // Note: r2r doesn't have explicit shutdown, so we just mark as not initialized
        self.is_initialized = false;
        
        log::info!("ROS interface shutdown complete");
        Ok(())
    }
    
    /// Get the current robot pose from odometry
    pub fn get_current_pose(&self) -> Option<Pose2D> {
        self.subscribers.odom.get_latest()
            .ok()
            .map(|odom| Pose2D {
                x: odom.pose.pose.position.x as f32,
                y: odom.pose.pose.position.y as f32,
                theta: 2.0 * (odom.pose.pose.orientation.z as f32).atan2(odom.pose.pose.orientation.w as f32),
            })
    }
}

/// ROS error types
#[derive(Debug)]
pub enum RosError {
    /// ROS initialization error
    InitError(String),
    /// Publishing error
    PublishError(String),
    /// Subscription error
    SubscribeError(String),
    /// Message conversion error
    ConversionError(String),
    /// Interface not initialized
    NotInitialized,
}

impl std::fmt::Display for RosError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            RosError::InitError(msg) => write!(f, "ROS init error: {}", msg),
            RosError::PublishError(msg) => write!(f, "Publish error: {}", msg),
            RosError::SubscribeError(msg) => write!(f, "Subscribe error: {}", msg),
            RosError::ConversionError(msg) => write!(f, "Conversion error: {}", msg),
            RosError::NotInitialized => write!(f, "ROS interface not initialized"),
        }
    }
}

impl std::error::Error for RosError {}

/// Sensor data collected from ROS
#[derive(Debug, Clone)]
pub struct SensorData {
    /// Laser scan data
    pub laser_scan: r2r::sensor_msgs::msg::LaserScan,
    /// IMU data
    pub imu_data: r2r::sensor_msgs::msg::Imu,
    /// Odometry data
    pub odom_data: r2r::nav_msgs::msg::Odometry,
}

/// Simple 2D pose representation
#[derive(Debug, Clone, Copy)]
pub struct Pose2D {
    /// X position
    pub x: f32,
    /// Y position
    pub y: f32,
    /// Orientation (theta)
    pub theta: f32,
}

/// Motion command for the robot
#[derive(Debug, Clone, Copy)]
pub struct MotionCommand {
    /// Linear velocity (m/s)
    pub linear: f32,
    /// Angular velocity (rad/s)
    pub angular: f32,
}

impl MotionCommand {
    /// Convert to ROS Twist message
    pub fn to_ros_message(&self) -> r2r::geometry_msgs::msg::Twist {
        r2r::geometry_msgs::msg::Twist {
            linear: r2r::geometry_msgs::msg::Vector3 {
                x: self.linear as f64,
                y: 0.0,
                z: 0.0,
            },
            angular: r2r::geometry_msgs::msg::Vector3 {
                x: 0.0,
                y: 0.0,
                z: self.angular as f64,
            },
        }
    }
}