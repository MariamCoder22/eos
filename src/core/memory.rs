// core/memory.rs

// Implements hippocampus-like navigation memory for Eos, storing topological maps,
// recent trajectories, and loop closures to support adaptive navigation. Serializes
// memory for persistence and supports loop closure detection for localization.

// Dependencies
use log::{error, info};
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::path::Path;
use super::localization::Pose;

// Node in topological map, representing a familiar location
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct MapNode {
    id: u64,
    pose: Pose,             // Position (x, y, theta)
    features: Vec<String>,  // Landmarks (e.g., "wall", "door")
    familiarity: f64,       // Confidence score [0, 1]
}

// Memory struct: Manages topological map, trajectory, and loop closures
#[derive(Clone, Serialize, Deserialize)]
pub struct Memory {
    topological_map: HashMap<u64, MapNode>, // Familiar locations
    trajectory: VecDeque<Pose>,             // Recent poses (last 100)
    loop_closures: Vec<(u64, u64)>,         // Pairs of revisited nodes
    node_counter: u64,                      // Incremental node IDs
}

impl Memory {
    /// Initializes memory with empty map and trajectory buffer
    pub fn new() -> Self {
        Memory {
            topological_map: HashMap::new(),
            trajectory: VecDeque::with_capacity(100),
            loop_closures: Vec::new(),
            node_counter: 0,
        }
    }

    /// Adds a new pose to the trajectory buffer
    pub fn add_pose(&mut self, pose: Pose) {
        if self.trajectory.len() >= 100 {
            self.trajectory.pop_front();
        }
        self.trajectory.push_back(pose);
    }

    /// Adds a new node to the topological map with detected features
    pub fn add_map_node(&mut self, pose: Pose, features: Vec<String>) -> u64 {
        let id = self.node_counter;
        self.node_counter += 1;
        let node = MapNode {
            id,
            pose,
            features,
            familiarity: 0.8, // Mock familiarity score
        };
        self.topological_map.insert(id, node);
        info!("Added map node {} at x={}, y={}", id, pose.x, pose.y);
        id
    }

    /// Checks for loop closure by comparing current pose to past nodes
    pub fn check_loop_closure(&self, current_pose: &Pose) -> Option<u64> {
        for (id, node) in &self.topological_map {
            let distance = ((current_pose.x - node.pose.x).powi(2)
                + (current_pose.y - node.pose.y).powi(2))
                .sqrt();
            if distance < 0.5 && (current_pose.theta - node.pose.theta).abs() < 0.1 {
                info!("Loop closure detected with node {}", id);
                return Some(*id);
            }
        }
        None
    }

    /// Serializes memory to a file for persistence
    pub fn save(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let file = File::create(path)?;
        serde_yaml::to_writer(file, self)?;
        info!("Saved memory to {}", path);
        Ok(())
    }

    /// Loads memory from a file
    pub fn load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let file = File::open(path)?;
        let memory: Memory = serde_yaml::from_reader(file)?;
        info!("Loaded memory from {}", path);
        Ok(memory)
    }

    /// Returns recent trajectory
    pub fn get_trajectory(&self) -> &VecDeque<Pose> {
        &self.trajectory
    }

    /// Returns topological map
    pub fn get_topological_map(&self) -> &HashMap<u64, MapNode> {
        &self.topological_map
    }
}

// Weaknesses:
// - Simplified loop closure detection (distance-based); lacks robust feature matching.
// Future improvement: Use ORB features or SNN-based place recognition for hippocampus-like memory.
// - Fixed-size trajectory buffer (100 poses); may need dynamic sizing for scalability.
// - Mock familiarity scores; needs integration with perception.rs for real feature data.
// - Serialization uses YAML, which may be slow for large maps; consider binary formats (e.g., bincode).
// - No multi-robot support; future versions should share maps across robots.

// Current Functionality:
// - Stores topological map with nodes (pose, features, familiarity).
// - Maintains a recent trajectory buffer (100 poses).
// - Detects loop closures using simple distance-based checks.
// - Serializes/loads memory to/from YAML files.
// - Provides access to trajectory and map for navigation and localization.
