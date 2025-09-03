use r2r::geometry_msgs::Twist;
use std::time::{Duration, Instant};

/// Advanced control system for rover movement
pub struct RoverControl {
    current_velocity: Twist,
    max_acceleration: f32,
    max_deceleration: f32,
    terrain_adaptation_factor: f32,
    safety_monitor: SafetyMonitor,
    last_command_time: Instant,
    command_history: Vec<(Twist, Instant)>,
}

pub struct SafetyMonitor {
    emergency_stop_triggered: bool,
    obstacle_proximity: f32,
    tilt_angle: f32,
    vibration_level: f32,
    safety_thresholds: SafetyThresholds,
}

pub struct SafetyThresholds {
    pub max_tilt: f32,
    pub max_vibration: f32,
    pub min_obstacle_distance: f32,
    pub max_acceleration: f32,
}

impl RoverControl {
    pub fn new() -> Self {
        RoverControl {
            current_velocity: Twist::default(),
            max_acceleration: 0.5,
            max_deceleration: 0.7,
            terrain_adaptation_factor: 1.0,
            safety_monitor: SafetyMonitor {
                emergency_stop_triggered: false,
                obstacle_proximity: 0.0,
                tilt_angle: 0.0,
                vibration_level: 0.0,
                safety_thresholds: SafetyThresholds {
                    max_tilt: 0.5,
                    max_vibration: 0.8,
                    min_obstacle_distance: 0.3,
                    max_acceleration: 0.5,
                },
            },
            last_command_time: Instant::now(),
            command_history: Vec::with_capacity(100),
        }
    }
    
    pub fn execute_movement(
        &mut self,
        path_segment: &PathSegment,
        terrain_analysis: &RoverTerrainAnalysis,
        energy_level: f32,
    ) -> Result<Twist, String> {
        // Check safety first
        if self.safety_monitor.emergency_stop_triggered {
            return Err("Emergency stop active".to_string());
        }
        
        // Calculate optimal velocity for this terrain
        let optimal_velocity = self.calculate_optimal_velocity(path_segment, terrain_analysis, energy_level);
        
        // Smooth acceleration towards optimal velocity
        let command = self.smooth_acceleration(optimal_velocity);
        
        // Update safety monitor
        self.update_safety_monitor(&command, terrain_analysis);
        
        // Record command
        self.record_command(command.clone());
        
        Ok(command)
    }
    
    fn calculate_optimal_velocity(
        &self,
        segment: &PathSegment,
        terrain: &RoverTerrainAnalysis,
        energy_level: f32,
    ) -> Twist {
        let mut velocity = Twist::default();
        
        // Base speed on terrain type and risk
        let base_speed = if let Some(profile) = self.get_terrain_profile(&segment.terrain_type) {
            profile.recommended_speed * (1.0 - segment.risk_factor)
        } else {
            0.5 * (1.0 - segment.risk_factor)
        };
        
        // Adjust for energy level
        let energy_factor = if energy_level > 0.7 {
            1.0
        } else if energy_level > 0.3 {
            0.7
        } else {
            0.4
        };
        
        velocity.linear.x = base_speed * energy_factor * self.terrain_adaptation_factor;
        
        velocity
    }
    
    fn smooth_acceleration(&mut self, target_velocity: Twist) -> Twist {
        let time_since_last = self.last_command_time.elapsed().as_secs_f32();
        let mut command = self.current_velocity.clone();
        
        // Calculate acceleration needed
        let speed_diff = target_velocity.linear.x - self.current_velocity.linear.x;
        let acceleration = if speed_diff > 0.0 {
            speed_diff.min(self.max_acceleration * time_since_last)
        } else {
            speed_diff.max(-self.max_deceleration * time_since_last)
        };
        
        command.linear.x = (self.current_velocity.linear.x + acceleration).max(0.0);
        
        // Update current velocity
        self.current_velocity = command.clone();
        self.last_command_time = Instant::now();
        
        command
    }
    
    fn update_safety_monitor(&mut self, command: &Twist, terrain: &RoverTerrainAnalysis) {
        // Update safety parameters
        self.safety_monitor.vibration_level = terrain.overall_difficulty;
        
        // Check for emergency conditions
        if self.safety_monitor.tilt_angle > self.safety_monitor.safety_thresholds.max_tilt ||
           self.safety_monitor.vibration_level > self.safety_monitor.safety_thresholds.max_vibration ||
           self.safety_monitor.obstacle_proximity < self.safety_monitor.safety_thresholds.min_obstacle_distance {
            self.safety_monitor.emergency_stop_triggered = true;
        }
        
        // Check acceleration limits
        if command.linear.x.abs() > self.safety_monitor.safety_thresholds.max_acceleration {
            self.safety_monitor.emergency_stop_triggered = true;
        }
    }
    
    fn record_command(&mut self, command: Twist) {
        if self.command_history.len() >= 100 {
            self.command_history.remove(0);
        }
        self.command_history.push((command, Instant::now()));
    }
    
    pub fn emergency_stop(&mut self) -> Twist {
        self.safety_monitor.emergency_stop_triggered = true;
        
        // Generate stop command with safe deceleration
        let mut stop_cmd = Twist::default();
        stop_cmd.linear.x = 0.0;
        
        // Record emergency stop
        self.record_command(stop_cmd.clone());
        
        stop_cmd
    }
    
    pub fn reset_safety(&mut self) {
        self.safety_monitor.emergency_stop_triggered = false;
    }
    
    pub fn adjust_terrain_adaptation(&mut self, factor: f32) {
        self.terrain_adaptation_factor = factor.max(0.1).min(2.0);
    }
    
    fn get_terrain_profile(&self, terrain_type: &str) -> Option<&TerrainProfile> {
        // This would reference the navigation module's terrain profiles
        // For now, return a default
        None
    }
    
    pub fn get_command_history(&self) -> &[(Twist, Instant)] {
        &self.command_history
    }
}