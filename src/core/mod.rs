// core/mod.rs

// Declares and exposes submodules for the Eos brainstem, providing a clean API for 
// perception, localization, and state management. This module ensures modularity and 
// controlled access to critical OS functions. 

// Expose submodules publicly for other Eos components (e.g., navigation, apps)
pub mod localization;
pub mod perception;
pub mod state;

// Re-export key types and functions for a unified API, minimizing external dependencies
pub use localization::{Localization, Pose, PoseConfidence};
pub use perception::{Perception, OccupancyGrid, SemanticObject, Snapshot};
pub use state::{CoreState, Mode};

// Imports for internal use
use log::error;
use r2r::QosProfile;
use std::sync::{Arc, Mutex};

// Core struct: Orchestrates brainstem functionality, integrating localization,
// perception, and state management
#[derive(Clone)]
pub struct Core {
    localization: Arc<Mutex<Localization>>,
    perception: Arc<Mutex<Perception>>,
    state: Arc<Mutex<CoreState>>,
}

impl Core {
    /// Initializes the brainstem with ROS 2 node, SNN, and configuration
    pub fn new(ros_node: &r2r::Node, config_path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let localization = Arc::new(Mutex::new(Localization::new(ros_node, config_path)?));
        let perception = Arc::new(Mutex::new(Perception::new(ros_node, config_path)?));
        let state = Arc::new(Mutex::new(CoreState::new()));

        Ok(Core {
            localization,
            perception,
            state,
        })
    }

    /// Updates the core state based on new sensor data and mode
    pub fn update(&self) -> Result<(), Box<dyn std::error::Error>> {
        let mut localization = self.localization.lock().unwrap();
        let mut perception = self.perception.lock().unwrap();
        let mut state = self.state.lock().unwrap();

        // Update localization with latest sensor data
        localization.update()?;
        // Update perception with new sensor snapshot
        perception.update()?;
        // Update state based on localization and perception
        state.update(&localization.get_current_pose(), &perception.get_snapshot())?;

        Ok(())
    }

    /// Returns the current robot pose
    pub fn get_pose(&self) -> Pose {
        self.localization.lock().unwrap().get_current_pose()
    }

    /// Returns the current perception snapshot
    pub fn get_perception_snapshot(&self) -> Snapshot {
        self.perception.lock().unwrap().get_snapshot()
    }

    /// Returns the current mode (navigate, relocalize, idle, panic)
    pub fn get_mode(&self) -> Mode {
        self.state.lock().unwrap().get_mode()
    }
}

// Weaknesses:
// - Thread safety relies on Arc<Mutex>, which may introduce contention in high-frequency updates.
// Future improvement: Use lock-free data structures or async Rust (tokio) for better concurrency.
// - Limited error propagation; detailed error types needed for production.
// - API is minimal; may need additional methods (e.g., reset, diagnostics) for apps/.
// - SNN integration is indirect (via perception.rs); direct SNN control here could enhance neuromorphic features.

// Current Functionality:
// - Initializes localization, perception, and state with ROS 2 integration.
// - Provides a unified API for pose, snapshot, and mode queries.
// - Updates all subsystems in a single call, ensuring consistency.
// - Thread-safe for concurrent access by navigation or apps.
