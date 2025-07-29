// core/api.rs

// Exposes a clean interface for external systems (CLI, dashboard, RViz) and internal
// modules to interact with Eos's core. Supports navigation commands, status queries,
// and logging hooks for monitoring.

// Dependencies
use log::{error, info};
use r2r::{geometry_msgs::msg::PoseStamped, QosProfile};
use std::sync::{Arc, Mutex};
use super::{localization::PoseConfidence, perception::Snapshot, state::Mode, Core};

// API struct: Wraps Core for external and internal access
#[derive(Clone)]
pub struct Api {
    core: Arc<Mutex<Core>>,
    ros_node: Arc<r2r::Node>,
    publisher: r2r::Publisher<PoseStamped>,
}

impl Api {
    /// Initializes API with Core and ROS 2 publisher
    pub fn new(core: Core, ros_node: &r2r::Node) -> Result<Self, Box<dyn std::error::Error>> {
        let publisher = ros_node.create_publisher::<PoseStamped>(
            "/cmd_pose",
            QosProfile::default(),
        )?;

        Ok(Api {
            core: Arc::new(Mutex::new(core)),
            ros_node: Arc::new(ros_node.clone()),
            publisher,
        })
    }

    /// Starts navigation to a goal pose
    pub fn start_navigation(&self, goal: PoseStamped) -> Result<(), Box<dyn std::error::Error>> {
        let mut core = self.core.lock().unwrap();

        if core.get_mode() == Mode::Idle || core.get_mode() == Mode::Recovering {
            self.publisher.publish(&goal)?;
            core.state.lock().unwrap().current_mode = Mode::Navigating;

            info!(
                "Started navigation to x={}, y={}",
                goal.pose.position.x, goal.pose.position.y
            );
            Ok(())
        } else {
            error!("Cannot start navigation in mode {:?}", core.get_mode());
            Err("Invalid state for navigation".into())
        }
    }

    /// Stops the robot
    pub fn stop(&self) -> Result<(), Box<dyn std::error::Error>> {
        let mut core = self.core.lock().unwrap();
        core.state.lock().unwrap().emergency_stop();
        info!("Robot stopped");
        Ok(())
    }

    /// Returns current status (mode, pose, snapshot)
    pub fn get_status(&self) -> (Mode, PoseConfidence, Snapshot) {
        let core = self.core.lock().unwrap();
        (
            core.get_mode(),
            core.get_pose(),
            core.get_perception_snapshot(),
        )
    }

    /// Queries current pose (for internal use)
    pub fn get_pose(&self) -> PoseConfidence {
        self.core.lock().unwrap().get_pose()
    }

    /// Queries current perception snapshot (for internal use)
    pub fn get_perception(&self) -> Snapshot {
        self.core.lock().unwrap().get_perception_snapshot()
    }
}

// Weaknesses:
// - Limited command set; lacks advanced controls (e.g., set_mode, configure_params).
// Future improvement: Add full CRUD API for modes and configurations.
// - ROS 2 publisher is basic; needs support for multiple topics (e.g., velocity).
// - No logging hooks for external dashboards; integrate with r2r’s logging services.
// - Mutex locking may cause contention in high-frequency calls.
// Future improvement: Use async Rust (tokio) for non-blocking API.
// - No CLI or Web UI integration; needs wrappers for external access.

// Current Functionality:
// - Provides navigation commands (start_navigation, stop) via ROS 2.
// - Exposes status, pose, and perception queries for internal/external use.
// - Ensures thread-safe access to Core via Arc<Mutex>.
// - Logs key events for debugging and monitoring.
