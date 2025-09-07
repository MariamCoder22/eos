// src/ros_interface/subscriber.rs
// Subscribes to ROS 2 /scan topic to receive LIDAR data for Eos navigation.

// Imports necessary dependencies and shared types.
// - r2r: ROS 2 Rust bindings for topic subscription.
// - SensorData: Defined in lib.rs for consistent data handling.
use r2r::{QosProfile, Node};
use crate::SensorData;

/// Subscriber struct to manage ROS 2 subscription and store latest data.
/// - `subscriber`: ROS 2 subscriber for LaserScan messages.
/// - `latest_data`: Stores the most recent sensor data for processing.
pub struct Subscriber {
    subscriber: r2r::Subscriber<r2r::sensor_msgs::msg::LaserScan>,
    latest_data: Option<SensorData>,
}

/// Implementation of Subscriber for creating and accessing data.
impl Subscriber {
    /// Creates a new subscriber for the given topic (e.g., `/scan`).
    /// - `node`: ROS 2 node for subscription.
    /// - `topic`: Topic name (e.g., `/scan` for LIDAR).
    /// Returns a `Subscriber` instance or an error if subscription fails.
    pub fn new(node: &mut Node, topic: &str) -> Result<Self, r2r::Error> {
        let qos = QosProfile::default(); // Default QoS for reliable delivery

        // Initialize with a placeholder callback to allow setup
        let mut subscriber = Self {
            subscriber: node.create_subscription::<r2r::sensor_msgs::msg::LaserScan, _>(
                topic,
                qos,
                |_| {}, // Temporary empty callback
            )?,
            latest_data: None,
        };

        // Reassign subscriber with closure that updates `latest_data`
        subscriber.subscriber = node.create_subscription::<r2r::sensor_msgs::msg::LaserScan, _>(
            topic,
            qos,
            move |msg| {
                // Convert ROS LaserScan to SensorData
                subscriber.latest_data = Some(SensorData {
                    ranges: msg.ranges.clone(),
                });
            },
        )?;

        Ok(subscriber)
    }

    /// Returns the latest sensor data, if available.
    /// Clones data to avoid ownership issues.
    pub fn get_data(&self) -> Option<SensorData> {
        self.latest_data.clone()
    }
}

// SWOT Analysis
// Strengths:
// - Robust Integration: Uses r2r for reliable ROS 2 subscription, compatible with TurtleBot3’s /scan topic.
// - Simplicity: Minimal code focused on LIDAR data, suitable for MVP demo in Gazebo.
// - Reusability: Generic structure supports other sensor types (e.g., cameras) in future iterations.
//
// Weaknesses:
// - Basic Callback: Simple closure may miss high-frequency data in real-world scenarios.
// - No Preprocessing: Directly converts LaserScan to SensorData without noise filtering or validation.
// - Error Handling: Minimal handling of subscription failures or invalid data.
//
// Opportunities:
// - Extensibility: Can add support for multiple sensors (e.g., /camera/image_raw) to enhance sensory processing.
// - Optimization: Async callbacks (e.g., tokio) could improve performance for real-time applications.
// - Z Fellows Appeal: Demonstrates ROS 2 proficiency, aligning with robotics focus for $10B market.
//
// Threats:
// - ROS 2 Dependency: Relies on r2r, which may have bugs or version incompatibilities.
// - Scalability Limits: Single-threaded subscription struggles with high data rates in complex environments.
