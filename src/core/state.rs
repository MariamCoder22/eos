// core/state.rs

// Manages the robot's high-level mode (Idle, Navigating, Lost, Recovering, Mapping)
// using a finite state machine. Handles mode transitions based on sensor/planner
// events and provides emergency overrides for safety.

// Dependencies
use log::{error, info};
use super::{localization::PoseConfidence, perception::Snapshot};

// Robot operating modes
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    Idle,        // Robot stopped, awaiting commands
    Navigating,  // Following a planned path
    Lost,        // High localization uncertainty
    Recovering,  // Attempting to relocalize
    Mapping,     // Building a new map
}

// Core state: Tracks mode and handles transitions
pub struct CoreState {
    current_mode: Mode,
    last_pose_confidence: f64,
    last_obstacle_distance: f64,
}

impl CoreState {
    /// Initializes state in Idle mode
    pub fn new() -> Self {
        CoreState {
            current_mode: Mode::Idle,
            last_pose_confidence: 1.0,         // Mock initial confidence
            last_obstacle_distance: f64::INFINITY, // No obstacles
        }
    }

    /// Updates state based on localization and perception data
    pub fn update(
        &mut self,
        pose_confidence: &PoseConfidence,
        snapshot: &Snapshot,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Extract confidence from covariance (simplified)
        let confidence = 1.0
            / (pose_confidence.covariance[(0, 0)] + pose_confidence.covariance[(1, 1)]).sqrt();
        self.last_pose_confidence = confidence;

        // Find nearest obstacle (simplified)
        self.last_obstacle_distance = snapshot
            .grid
            .data
            .iter()
            .enumerate()
            .filter(|(_, &cell)| cell == 1)
            .map(|(i, _)| {
                let x = (i % snapshot.grid.width) as f64 * snapshot.grid.resolution;
                let y = (i / snapshot.grid.width) as f64 * snapshot.grid.resolution;
                (x.powi(2) + y.powi(2)).sqrt()
            })
            .fold(f64::INFINITY, f64::min);

        // Mode transition logic
        match self.current_mode {
            Mode::Idle => {
                if confidence < 0.5 {
                    self.current_mode = Mode::Lost;
                    error!("Transitioned to Lost: low confidence ({})", confidence);
                }
            }
            Mode::Navigating => {
                if confidence < 0.5 {
                    self.current_mode = Mode::Lost;
                    error!("Transitioned to Lost: low confidence ({})", confidence);
                } else if self.last_obstacle_distance < 0.3 {
                    self.current_mode = Mode::Recovering;
                    error!(
                        "Transitioned to Recovering: obstacle too close ({})",
                        self.last_obstacle_distance
                    );
                }
            }
            Mode::Lost => {
                if confidence > 0.8 {
                    self.current_mode = Mode::Recovering;
                    info!("Transitioned to Recovering: confidence improved ({})", confidence);
                }
            }
            Mode::Recovering => {
                if confidence > 0.9 {
                    self.current_mode = Mode::Navigating;
                    info!("Transitioned to Navigating: confidence restored ({})", confidence);
                }
            }
            Mode::Mapping => {
                if confidence < 0.5 {
                    self.current_mode = Mode::Lost;
                    error!(
                        "Transitioned to Lost: low confidence during mapping ({})",
                        confidence
                    );
                }
            }
        }

        Ok(())
    }

    /// Emergency override: Stops robot if lost or in danger
    pub fn emergency_stop(&mut self) {
        self.current_mode = Mode::Idle;
        error!("Emergency stop triggered");
    }

    /// Returns current mode
    pub fn get_mode(&self) -> Mode {
        self.current_mode
    }
}

// Weaknesses:
// - Simplified FSM; lacks complex transitions (e.g., timeouts, multi-step recovery).
// Future improvement: Use hierarchical FSM or behavior trees for richer logic.
// - Mock confidence calculation; needs robust covariance analysis.
// - Obstacle distance is basic; integrate perception.rs semantic objects for smarter decisions.
// - No event queue for sensor/planner inputs; risks missing transient events.
// Future improvement: Add tokio::sync::mpsc for asynchronous event handling.
// - Limited modes; could add Exploration, Charging, etc., for extensibility.

// Current Functionality:
// - Maintains FSM with five modes (Idle, Navigating, Lost, Recovering, Mapping).
// - Updates mode based on pose confidence and obstacle proximity.
// - Supports emergency stop for safety.
// - Provides mode query for navigation and API.
