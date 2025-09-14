//! Navigation system for Eos OS
//!
//! This module handles path planning, obstacle avoidance, and motion control
//! based on sensor data and neural network outputs.

use std::collections::VecDeque;

/// Navigation planner for path planning and obstacle avoidance
pub struct NavigationPlanner {
    config: NavigationConfig,
    path_history: VecDeque<PathSegment>,
    obstacle_map: Vec<Obstacle>,
    current_goal: Option<Pose2D>,
    safety_monitor: SafetyMonitor,
}

/// Motion controller for executing navigation plans
pub struct MotionController {
    config: NavigationConfig,
    motion_profile: MotionProfile,
    command_history: VecDeque<MotionCommand>,
    safety_limits: SafetyLimits,
}

/// Navigation configuration
#[derive(Debug, Clone)]
pub struct NavigationConfig {
    /// Maximum linear velocity
    pub max_linear_velocity: f32,
    /// Maximum angular velocity
    pub max_angular_velocity: f32,
    /// Maximum acceleration
    pub max_acceleration: f32,
    /// Safety distance from obstacles
    pub safety_distance: f32,
    /// Goal tolerance
    pub goal_tolerance: f32,
    /// Obstacle inflation radius
    pub obstacle_inflation: f32,
}

/// Navigation status
#[derive(Debug, Clone)]
pub struct NavigationStatus {
    /// Whether a goal is set
    pub has_goal: bool,
    /// Distance to goal
    pub distance_to_goal: f32,
    /// Number of detected obstacles
    pub obstacle_count: usize,
    /// Safety status
    pub safety_status: SafetyStatus,
    /// Current motion state
    pub motion_state: MotionState,
}

/// Safety monitor for navigation
#[derive(Debug, Clone)]
struct SafetyMonitor {
    /// Minimum safe distance
    min_safe_distance: f32,
    /// Emergency stop flag
    emergency_stop: bool,
    /// Safety violations count
    safety_violations: u32,
}

/// Motion profile for smooth control
#[derive(Debug, Clone)]
struct MotionProfile {
    /// Current velocity
    current_velocity: MotionCommand,
    /// Target velocity
    target_velocity: MotionCommand,
    /// Acceleration limits
    acceleration_limits: MotionCommand,
}

/// Safety limits for motion control
#[derive(Debug, Clone)]
struct SafetyLimits {
    /// Maximum allowed velocity
    max_velocity: MotionCommand,
    /// Maximum allowed acceleration
    max_acceleration: MotionCommand,
    /// Emergency stop deceleration
    emergency_deceleration: f32,
}

/// Path segment for navigation
#[derive(Debug, Clone)]
struct PathSegment {
    /// Start pose
    start: Pose2D,
    /// End pose
    end: Pose2D,
    /// Segment length
    length: f32,
    /// Segment safety score
    safety_score: f32,
}

/// Obstacle representation
#[derive(Debug, Clone)]
struct Obstacle {
    /// Position
    position: Pose2D,
    /// Radius
    radius: f32,
    /// Confidence
    confidence: f32,
    /// Velocity (if moving)
    velocity: Option<(f32, f32)>,
}

/// Safety status
#[derive(Debug, Clone, PartialEq)]
enum SafetyStatus {
    /// Normal operation
    Normal,
    /// Warning - approaching limits
    Warning,
    /// Critical - safety limits exceeded
    Critical,
    /// Emergency stop active
    EmergencyStop,
}

/// Motion state
#[derive(Debug, Clone, PartialEq)]
enum MotionState {
    /// Stopped
    Stopped,
    /// Moving
    Moving,
    /// Avoiding obstacle
    Avoiding,
    /// Approaching goal
    ApproachingGoal,
    /// Emergency stopping
    EmergencyStopping,
}

impl NavigationPlanner {
    /// Create a new navigation planner
    pub fn new(config: &NavigationConfig) -> Self {
        NavigationPlanner {
            config: config.clone(),
            path_history: VecDeque::with_capacity(100),
            obstacle_map: Vec::new(),
            current_goal: None,
            safety_monitor: SafetyMonitor {
                min_safe_distance: config.safety_distance,
                emergency_stop: false,
                safety_violations: 0,
            },
        }
    }
    
    /// Plan a path based on sensor data and neural output
    pub fn plan(
        &mut self,
        sensor_data: &super::ros_interface::SensorData,
        neural_output: &[f32],
        current_pose: Option<Pose2D>,
    ) -> Result<Path, NavigationError> {
        // Update obstacle map from sensor data
        self.update_obstacle_map(sensor_data);
        
        // Apply neural network guidance
        self.apply_neural_guidance(neural_output);
        
        // Get current pose or use default
        let current_pose = current_pose.unwrap_or(Pose2D { x: 0.0, y: 0.0, theta: 0.0 });
        
        // Plan path to goal
        let path = if let Some(goal) = self.current_goal {
            self.plan_path_to_goal(current_pose, goal)
        } else {
            // No goal set, perform exploration
            self.plan_exploration_path(current_pose)
        };
        
        // Check safety
        self.check_safety(&path);
        
        // Store path history
        if let Ok(path) = &path {
            self.store_path_history(path);
        }
        
        path
    }
    
    /// Set a new navigation goal
    pub fn set_goal(&mut self, goal: Pose2D) {
        self.current_goal = Some(goal);
        log::info!("New navigation goal set: {:?}", goal);
    }
    
    /// Clear the current goal
    pub fn clear_goal(&mut self) {
        self.current_goal = None;
        log::info!("Navigation goal cleared");
    }
    
    /// Get current navigation status
    pub fn get_status(&self) -> NavigationStatus {
        let distance_to_goal = self.current_goal
            .map(|goal| self.calculate_distance(goal, Pose2D { x: 0.0, y: 0.0, theta: 0.0 }))
            .unwrap_or(0.0);
            
        NavigationStatus {
            has_goal: self.current_goal.is_some(),
            distance_to_goal,
            obstacle_count: self.obstacle_map.len(),
            safety_status: if self.safety_monitor.emergency_stop {
                SafetyStatus::EmergencyStop
            } else if self.safety_monitor.safety_violations > 0 {
                SafetyStatus::Warning
            } else {
                SafetyStatus::Normal
            },
            motion_state: MotionState::Stopped, // This would be updated by motion controller
        }
    }
    
    /// Update obstacle map from sensor data
    fn update_obstacle_map(&mut self, sensor_data: &super::ros_interface::SensorData) {
        self.obstacle_map.clear();
        
        // Process laser scan data for obstacles
        let scan = &sensor_data.laser_scan;
        for (i, range) in scan.ranges.iter().enumerate() {
            if *range < scan.range_max && *range > scan.range_min {
                let angle = scan.angle_min + (i as f32) * scan.angle_increment;
                let x = range * angle.cos();
                let y = range * angle.sin();
                
                self.obstacle_map.push(Obstacle {
                    position: Pose2D { x: x as f32, y: y as f32, theta: 0.0 },
                    radius: self.config.obstacle_inflation,
                    confidence: 0.8, // Default confidence
                    velocity: None, // Would be calculated from multiple scans
                });
            }
        }
    }
    
    /// Apply neural network guidance to navigation
    fn apply_neural_guidance(&mut self, neural_output: &[f32]) {
        // Simple guidance application - would be more complex in production
        if neural_output.len() >= 2 {
            // First output: preference for forward movement (0-1)
            // Second output: preference for turning (-1 to 1)
            let forward_bias = neural_output[0];
            let turn_bias = neural_output[1] * 2.0 - 1.0; // Convert from 0-1 to -1 to 1
            
            // Adjust safety distance based on neural output
            if neural_output.len() >= 3 {
                let caution_level = neural_output[2];
                self.safety_monitor.min_safe_distance = self.config.safety_distance * (0.5 + caution_level * 0.5);
            }
            
            log::debug!("Neural guidance - forward: {:.2}, turn: {:.2}", forward_bias, turn_bias);
        }
    }
    
    /// Plan a path to a specific goal
    fn plan_path_to_goal(&self, start: Pose2D, goal: Pose2D) -> Result<Path, NavigationError> {
        // Simple straight-line path planning with obstacle avoidance
        // Would use more advanced algorithms in production
        
        let distance = self.calculate_distance(start, goal);
        let direction = self.calculate_direction(start, goal);
        
        // Check for obstacles along the path
        let safety_score = self.calculate_path_safety(start, goal);
        
        if safety_score < 0.3 {
            return Err(NavigationError::UnsafePath(
                format!("Path to goal is unsafe (score: {:.2})", safety_score)
            ));
        }
        
        Ok(Path {
            segments: vec![PathSegment {
                start,
                end: goal,
                length: distance,
                safety_score,
            }],
            total_length: distance,
            overall_safety: safety_score,
        })
    }
    
    /// Plan an exploration path
    fn plan_exploration_path(&self, current_pose: Pose2D) -> Result<Path, NavigationError> {
        // Simple exploration: move forward while avoiding obstacles
        let exploration_distance = 2.0; // meters
        
        let goal = Pose2D {
            x: current_pose.x + exploration_distance * current_pose.theta.cos(),
            y: current_pose.y + exploration_distance * current_pose.theta.sin(),
            theta: current_pose.theta,
        };
        
        self.plan_path_to_goal(current_pose, goal)
    }
    
    /// Check path safety
    fn check_safety(&mut self, path: &Path) {
        for segment in &path.segments {
            for obstacle in &self.obstacle_map {
                let distance = self.calculate_distance(obstacle.position, segment.start);
                
                if distance < self.safety_monitor.min_safe_distance + obstacle.radius {
                    self.safety_monitor.safety_violations += 1;
                    log::warn!("Safety violation: obstacle too close ({:.2}m)", distance);
                    
                    if distance < self.config.safety_distance * 0.5 {
                        self.safety_monitor.emergency_stop = true;
                        log::error!("EMERGENCY STOP: obstacle dangerously close ({:.2}m)", distance);
                    }
                }
            }
        }
    }
    
    /// Calculate distance between two poses
    fn calculate_distance(&self, a: Pose2D, b: Pose2D) -> f32 {
        ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()
    }
    
    /// Calculate direction from start to end
    fn calculate_direction(&self, start: Pose2D, end: Pose2D) -> f32 {
        (end.y - start.y).atan2(end.x - start.x)
    }
    
    /// Calculate safety score for a path segment
    fn calculate_path_safety(&self, start: Pose2D, end: Pose2D) -> f32 {
        let mut min_distance = f32::MAX;
        
        for obstacle in &self.obstacle_map {
            // Simple distance-based safety calculation
            // Would use more sophisticated collision checking in production
            let distance = self.calculate_distance(obstacle.position, start);
            min_distance = min_distance.min(distance);
        }
        
        // Convert distance to safety score (0-1)
        (min_distance / (self.config.safety_distance * 2.0)).min(1.0)
    }
    
    /// Store path in history
    fn store_path_history(&mut self, path: &Path) {
        for segment in &path.segments {
            self.path_history.push_back(segment.clone());
            if self.path_history.len() > 100 {
                self.path_history.pop_front();
            }
        }
    }
}

impl MotionController {
    /// Create a new motion controller
    pub fn new(config: &NavigationConfig) -> Self {
        MotionController {
            config: config.clone(),
            motion_profile: MotionProfile {
                current_velocity: MotionCommand { linear: 0.0, angular: 0.0 },
                target_velocity: MotionCommand { linear: 0.0, angular: 0.0 },
                acceleration_limits: MotionCommand { 
                    linear: config.max_acceleration, 
                    angular: config.max_acceleration,
                },
            },
            command_history: VecDeque::with_capacity(100),
            safety_limits: SafetyLimits {
                max_velocity: MotionCommand { 
                    linear: config.max_linear_velocity, 
                    angular: config.max_angular_velocity,
                },
                max_acceleration: MotionCommand { 
                    linear: config.max_acceleration, 
                    angular: config.max_acceleration,
                },
                emergency_deceleration: config.max_acceleration * 2.0,
            },
        }
    }
    
    /// Execute a navigation plan
    pub fn execute_plan(&mut self, plan: &Path) -> Result<MotionCommand, NavigationError> {
        if plan.segments.is_empty() {
            return Ok(MotionCommand { linear: 0.0, angular: 0.0 });
        }
        
        // For simplicity, use the first segment
        let segment = &plan.segments[0];
        
        // Calculate desired velocity based on segment
        let desired_velocity = self.calculate_desired_velocity(segment);
        
        // Apply motion profile to smooth velocity changes
        let smoothed_velocity = self.apply_motion_profile(desired_velocity);
        
        // Check safety limits
        if !self.check_velocity_limits(smoothed_velocity) {
            return Err(NavigationError::VelocityLimitExceeded);
        }
        
        // Store command history
        self.command_history.push_back(smoothed_velocity);
        if self.command_history.len() > 100 {
            self.command_history.pop_front();
        }
        
        Ok(smoothed_velocity)
    }
    
    /// Calculate desired velocity for a path segment
    fn calculate_desired_velocity(&self, segment: &PathSegment) -> MotionCommand {
        // Simple velocity calculation based on segment length and safety
        let base_speed = self.config.max_linear_velocity;
        let safety_factor = segment.safety_score;
        
        MotionCommand {
            linear: base_speed * safety_factor,
            angular: 0.0, // Would calculate based on curvature in production
        }
    }
    
    /// Apply motion profile to smooth velocity changes
    fn apply_motion_profile(&mut self, desired_velocity: MotionCommand) -> MotionCommand {
        // Simple linear acceleration limiting
        let max_delta_linear = self.motion_profile.acceleration_limits.linear * 0.1; // Assuming 100ms cycle
        let max_delta_angular = self.motion_profile.acceleration_limits.angular * 0.1;
        
        let delta_linear = (desired_velocity.linear - self.motion_profile.current_velocity.linear)
            .clamp(-max_delta_linear, max_delta_linear);
            
        let delta_angular = (desired_velocity.angular - self.motion_profile.current_velocity.angular)
            .clamp(-max_delta_angular, max_delta_angular);
            
        let new_velocity = MotionCommand {
            linear: self.motion_profile.current_velocity.linear + delta_linear,
            angular: self.motion_profile.current_velocity.angular + delta_angular,
        };
        
        // Update current velocity
        self.motion_profile.current_velocity = new_velocity;
        
        new_velocity
    }
    
    /// Check if velocity is within safety limits
    fn check_velocity_limits(&self, velocity: MotionCommand) -> bool {
        velocity.linear.abs() <= self.safety_limits.max_velocity.linear &&
        velocity.angular.abs() <= self.safety_limits.max_velocity.angular
    }
    
    /// Perform emergency stop
    pub fn emergency_stop(&mut self) -> MotionCommand {
        // Apply emergency deceleration
        self.motion_profile.current_velocity.linear = 0.0;
        self.motion_profile.current_velocity.angular = 0.0;
        
        MotionCommand { linear: 0.0, angular: 0.0 }
    }
}

/// Path representation for navigation
#[derive(Debug, Clone)]
pub struct Path {
    /// Path segments
    segments: Vec<PathSegment>,
    /// Total path length
    total_length: f32,
    /// Overall safety score
    overall_safety: f32,
}

/// Navigation error types
#[derive(Debug)]
pub enum NavigationError {
    /// No path found to goal
    NoPathError(String),
    /// Path is unsafe
    UnsafePath(String),
    /// Velocity limits exceeded
    VelocityLimitExceeded,
    /// Invalid goal
    InvalidGoal,
    /// Planning timeout
    PlanningTimeout,
}

impl std::fmt::Display for NavigationError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            NavigationError::NoPathError(msg) => write!(f, "No path found: {}", msg),
            NavigationError::UnsafePath(msg) => write!(f, "Unsafe path: {}", msg),
            NavigationError::VelocityLimitExceeded => write!(f, "Velocity limit exceeded"),
            NavigationError::InvalidGoal => write!(f, "Invalid goal"),
            NavigationError::PlanningTimeout => write!(f, "Planning timeout"),
        }
    }
}

impl std::error::Error for NavigationError {}