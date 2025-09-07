// src/neural/snn.rs
// Implements a simplified spiking neural network for navigation decisions.

// Imports shared types from lib.rs.
use crate::{SensorData, NavCommand};
use log::info;

/// SNN struct to simulate neuromorphic processing.
/// - `neurons`: Number of neurons (fixed for MVP).
/// - `weights`: Placeholder for synaptic weights (future learning).
pub struct Snn {
    neurons: usize,
    weights: Vec<f32>, // Simulated weights for MVP
}

impl Snn {
    /// Creates a new SNN with the given number of neurons.
    pub fn new(neurons: usize) -> Self {
        // Initialize basic weights (equal for simplicity)
        let weights = vec![1.0; neurons];
        Snn { neurons, weights }
    }

    /// Processes sensor data to produce a navigation command.
    /// - `data`: LIDAR ranges from Subscriber.
    /// Returns a `NavCommand` based on simple obstacle avoidance logic.
    pub fn process(&self, data: &SensorData) -> NavCommand {
        // Simulate SNN: Check for close obstacles
        let min_distance = data
            .ranges
            .iter()
            .filter(|&&d| d > 0.0)
            .fold(f32::INFINITY, |a, &b| a.min(b));

        info!("SNN processed data, min_distance: {}", min_distance);

        // Basic decision: Turn if obstacle near, else move forward
        if min_distance < 0.5 {
            NavCommand {
                linear: 0.0,
                angular: 0.5,
            } // Turn right
        } else {
            NavCommand {
                linear: 0.2,
                angular: 0.0,
            } // Move forward
        }
    }
}

// SWOT Analysis
// Strengths:
// - Simplicity: Basic SNN logic compiles and works for MVP demo in Gazebo.
// - Neuromorphic Vision: Mimics brain-inspired processing, aligning with Z Fellows’ sci-fi focus.
// - Logging: Uses env_logger for debugging, aiding development.
//
// Weaknesses:
// - Placeholder Logic: No real spiking neurons or learning (e.g., STDP), limiting memory simulation.
// - Limited Realism: Simple distance-based decision doesn’t reflect true neuromorphic behavior.
// - Scalability: Fixed weights prevent adaptive learning in MVP.
//
// Opportunities:
// - Real SNN: Integrate NEST or Brian2 post-MVP for true spiking neural networks.
// - Learning: Add synaptic plasticity for memory retention, enhancing demo impact.
// - Traction: Open-source SNN attracts developers, supporting 100+ GitHub stars goal.
//
// Threats:
// - Technical Gap: Lack of true SNN may underwhelm Z Fellows’ technical reviewers.
// - Complexity: Adding real SNN (e.g., NEST bindings) risks delays beyond August 20.
// - Competition: Established neuromorphic frameworks (e.g., Intel’s Loihi) are more advanced.
