// src/main.rs
// Entry point for Eos, coordinating SNN, ROS 2, and navigation for adaptive robotics.

// Imports dependencies and Eos modules.
// - r2r: ROS 2 bindings for node and topic management.
// - env_logger: Logging for debugging.
// - Eos modules: Neural, ROS interface, and navigation components.
use r2r::{QosProfile, Node, Context};
use log::{info, error};
use env_logger;
use eos_os::{
    neural::Snn,
    ros_interface::{Publisher, Subscriber},
    navigation::Planner,
};
use std::error::Error;

/// Main function to initialize and run Eos.
/// Coordinates sensory input, SNN processing, and navigation output.
fn main() -> Result<(), Box<dyn Error>> {
    // Initialize logging for debugging
    env_logger::init();
    info!("Starting Eos neuromorphic OS...");

    // Create ROS 2 context and node
    let ctx = Context::create()?;
    let mut node = Node::create(&ctx, "eos_node", "")?;
    info!("ROS 2 node initialized");

    // Initialize SNN with 100 neurons
    let snn = Snn::new(100);

    // Set up subscriber for LIDAR data (/scan)
    let subscriber = Subscriber::new(&mut node, "/scan")?;
    info!("Subscribed to /scan");

    // Set up publisher for navigation commands (/cmd_vel)
    let publisher = Publisher::new(&mut node, "/cmd_vel")?;
    info!("Publishing to /cmd_vel");

    // Initialize navigation planner with SNN
    let planner = Planner::new(snn);

    // Main loop: Process sensor data and publish commands
    // Runs until interrupted (Ctrl+C in production)
    let mut iteration = 0;
    while iteration < 10 {
        // Limited for MVP demo

        // Spin node to process ROS 2 messages
        node.spin_once(std::time::Duration::from_millis(100))?;

        // Process sensor data if available
        if let Some(sensor_data) = subscriber.get_data() {
            info!("Received LIDAR data: {} ranges", sensor_data.ranges.len());

            // Plan navigation using SNN
            let cmd = planner.plan(&sensor_data);

            // Publish navigation command
            if let Err(e) = publisher.publish(&cmd) {
                error!("Failed to publish command: {}", e);
            } else {
                info!(
                    "Published command: linear={}, angular={}",
                    cmd.linear, cmd.angular
                );
            }
        }

        // Simulate processing delay
        std::thread::sleep(std::time::Duration::from_millis(1000));
        iteration += 1;
    }

    info!("Eos demo completed");
    Ok(())
}

// SWOT Analysis
// Strengths:
// - Integration: Coordinates sensory, memory, and spatial components, showing Eos’s holistic approach.
// - Demo-Ready: Functional loop produces Gazebo-compatible output for Z Fellows video.
// - Robust Logging: env_logger aids debugging, ensuring reliable demo.
//
// Weaknesses:
// - Single-Threaded: Polling loop limits real-time performance; async needed for production.
// - Basic Error Handling: Logs errors but doesn’t recover from failures (e.g., ROS node crash).
// - Limited Iterations: Fixed loop (10 iterations) restricts demo duration.
//
// Opportunities:
// - Async Upgrade: Use tokio for non-blocking ROS 2 processing, improving scalability.
// - Traction: Clear code attracts developers for 100+ GitHub stars, boosting Z Fellows application.
// - Expansion: Add multi-sensor support or complex reasoning for future iterations.
//
// Threats:
// - Complexity: Adding async or error recovery risks missing August 20 deadline.
// - ROS 2 Dependency: r2r issues (e.g., version conflicts) could break integration.
// - Competition: ROS 2’s Nav2 stack is more mature, potentially overshadowing Eos’s simplicity.
