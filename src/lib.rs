//! Eos Robotics OS - Humanistic Robotics Operating System
//! 
//! This library provides the core functionality for the Eos robotics operating system,
//! including neural network integration, ROS 2 interface, and navigation systems.

#![warn(missing_docs)]
#![warn(unused_extern_crates)]

pub mod core;
pub mod neural;
pub mod ros_interface;
pub mod navigation;
pub mod apps;

// Re-export commonly used items for easier access
pub use core::{Localizer, SpatialMemory, PerceptionEngine, StateController, SystemAPI};
pub use neural::{SNNEngine, NeuralConfig};
pub use ros_interface::{RosInterface, Publisher, Subscriber};
pub use navigation::{NavigationPlanner, MotionController};

/// Main configuration structure for Eos OS
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EosConfig {
    /// Neural network configuration
    pub neural_config: NeuralConfig,
    /// ROS 2 configuration
    pub ros_config: RosConfig,
    /// Navigation parameters
    pub navigation_config: NavigationConfig,
    /// Core system settings
    pub core_config: core::CoreConfig,
}

/// ROS 2 specific configuration
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RosConfig {
    /// ROS domain ID
    pub domain_id: u32,
    /// Node name
    pub node_name: String,
    /// QoS settings
    pub qos_depth: usize,
}

/// Navigation system configuration
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NavigationConfig {
    /// Default maximum velocity
    pub max_velocity: f32,
    /// Default acceleration
    pub max_acceleration: f32,
    /// Safety distance from obstacles
    pub safety_distance: f32,
    /// Goal tolerance
    pub goal_tolerance: f32,
}

impl Default for EosConfig {
    fn default() -> Self {
        EosConfig {
            neural_config: NeuralConfig::default(),
            ros_config: RosConfig {
                domain_id: 0,
                node_name: "eos_robot".to_string(),
                qos_depth: 10,
            },
            navigation_config: NavigationConfig {
                max_velocity: 0.5,
                max_acceleration: 0.3,
                safety_distance: 0.5,
                goal_tolerance: 0.1,
            },
            core_config: core::CoreConfig::default(),
        }
    }
}

/// Primary entry point for Eos OS
pub struct EosOS {
    config: EosConfig,
    ros_interface: RosInterface,
    neural_engine: SNNEngine,
    navigation_planner: NavigationPlanner,
    motion_controller: MotionController,
    is_initialized: bool,
}

impl EosOS {
    /// Create a new Eos OS instance with the given configuration
    pub fn new(config: EosConfig) -> Result<Self, EosError> {
        let ros_interface = RosInterface::new(&config.ros_config)?;
        let neural_engine = SNNEngine::new(&config.neural_config)?;
        let navigation_planner = NavigationPlanner::new(&config.navigation_config);
        let motion_controller = MotionController::new(&config.navigation_config);
        
        Ok(EosOS {
            config,
            ros_interface,
            neural_engine,
            navigation_planner,
            motion_controller,
            is_initialized: false,
        })
    }
    
    /// Initialize all Eos OS components
    pub fn initialize(&mut self) -> Result<(), EosError> {
        log::info!("Initializing Eos OS...");
        
        // Initialize ROS interface
        self.ros_interface.initialize()?;
        
        // Initialize neural engine
        self.neural_engine.initialize()?;
        
        // Load any pre-trained models
        self.neural_engine.load_model("models/default_snn.json")
            .map_err(|e| EosError::NeuralError(e.to_string()))?;
            
        self.is_initialized = true;
        log::info!("Eos OS initialized successfully");
        
        Ok(())
    }
    
    /// Main execution loop for Eos OS
    pub fn run_cycle(&mut self) -> Result<(), EosError> {
        if !self.is_initialized {
            return Err(EosError::NotInitialized);
        }
        
        // Get sensor data from ROS
        let sensor_data = self.ros_interface.get_sensor_data()?;
        
        // Process sensor data with neural network
        let neural_output = self.neural_engine.process(&sensor_data)?;
        
        // Plan navigation based on neural output
        let navigation_plan = self.navigation_planner.plan(
            &sensor_data, 
            &neural_output,
            self.ros_interface.get_current_pose()
        )?;
        
        // Execute the motion plan
        let motion_command = self.motion_controller.execute_plan(&navigation_plan)?;
        
        // Publish motion commands to ROS
        self.ros_interface.publish_command(&motion_command)?;
        
        Ok(())
    }
    
    /// Shutdown Eos OS gracefully
    pub fn shutdown(&mut self) -> Result<(), EosError> {
        log::info!("Shutting down Eos OS...");
        
        // Save neural network state
        self.neural_engine.save_model("models/snn_state.json")
            .map_err(|e| EosError::NeuralError(e.to_string()))?;
            
        // Shutdown ROS interface
        self.ros_interface.shutdown()?;
        
        self.is_initialized = false;
        log::info!("Eos OS shutdown complete");
        
        Ok(())
    }
    
    /// Get current system status
    pub fn get_status(&self) -> SystemStatus {
        // This would collect status from all components
        SystemStatus {
            neural: self.neural_engine.get_status(),
            navigation: self.navigation_planner.get_status(),
            ros: self.ros_interface.get_status(),
            operational: self.is_initialized,
        }
    }
}

/// Eos OS error types
#[derive(Debug)]
pub enum EosError {
    /// ROS interface error
    RosError(String),
    /// Neural network error
    NeuralError(String),
    /// Navigation error
    NavigationError(String),
    /// System not initialized
    NotInitialized,
    /// Configuration error
    ConfigError(String),
}

impl std::fmt::Display for EosError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            EosError::RosError(msg) => write!(f, "ROS error: {}", msg),
            EosError::NeuralError(msg) => write!(f, "Neural error: {}", msg),
            EosError::NavigationError(msg) => write!(f, "Navigation error: {}", msg),
            EosError::NotInitialized => write!(f, "System not initialized"),
            EosError::ConfigError(msg) => write!(f, "Configuration error: {}", msg),
        }
    }
}

impl std::error::Error for EosError {}

/// Combined system status
#[derive(Debug, Clone)]
pub struct SystemStatus {
    /// Neural engine status
    pub neural: neural::NeuralStatus,
    /// Navigation system status
    pub navigation: navigation::NavigationStatus,
    /// ROS interface status
    pub ros: ros_interface::RosStatus,
    /// Overall operational status
    pub operational: bool,
}
