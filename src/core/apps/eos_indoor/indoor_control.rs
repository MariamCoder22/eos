use r2r::geometry_msgs::Twist;
use std::time::{Duration, Instant};

/// Indoor control system with social awareness and human interaction
pub struct IndoorControl {
    current_velocity: Twist,
    max_acceleration: f32,
    max_deceleration: f32,
    social_awareness_factor: f32,
    safety_monitor: IndoorSafetyMonitor,
    last_command_time: Instant,
    command_history: Vec<(Twist, Instant)>,
    approach_behavior: ApproachBehavior,
}

pub struct IndoorSafetyMonitor {
    emergency_stop_triggered: bool,
    human_proximity: f32,
    social_discomfort: f32,
    privacy_violation: f32,
    safety_thresholds: IndoorSafetyThresholds,
}

pub struct IndoorSafetyThresholds {
    pub max_human_proximity: f32,
    pub max_social_discomfort: f32,
    pub max_privacy_violation: f32,
    pub max_acceleration: f32,
}

pub enum ApproachBehavior {
    Avoidant,
    Neutral,
    Friendly,
    Assertive,
}

impl IndoorControl {
    pub fn new() -> Self {
        IndoorControl {
            current_velocity: Twist::default(),
            max_acceleration: 0.2,
            max_deceleration: 0.3,
            social_awareness_factor: 0.8,
            safety_monitor: IndoorSafetyMonitor {
                emergency_stop_triggered: false,
                human_proximity: 0.0,
                social_discomfort: 0.0,
                privacy_violation: 0.0,
                safety_thresholds: IndoorSafetyThresholds {
                    max_human_proximity: 0.8,
                    max_social_discomfort: 0.7,
                    max_privacy_violation: 0.6,
                    max_acceleration: 0.3,
                },
            },
            last_command_time: Instant::now(),
            command_history: Vec::with_capacity(100),
            approach_behavior: ApproachBehavior::Neutral,
        }
    }
    
    pub fn execute_movement(
        &mut self,
        path_segment: &IndoorPathSegment,
        human_analysis: &HumanPresenceAnalysis,
        energy_level: f32,
    ) -> Result<Twist, String> {
        // Check safety first
        if self.safety_monitor.emergency_stop_triggered {
            return Err("Emergency stop active".to_string());
        }
        
        // Calculate optimal velocity for this segment
        let optimal_velocity = self.calculate_optimal_velocity(path_segment, human_analysis, energy_level);
        
        // Smooth acceleration towards optimal velocity
        let command = self.smooth_acceleration(optimal_velocity);
        
        // Update safety monitor
        self.update_safety_monitor(&command, path_segment, human_analysis);
        
        // Record command
        self.record_command(command.clone());
        
        Ok(command)
    }
    
    fn calculate_optimal_velocity(
        &self,
        segment: &IndoorPathSegment,
        human_analysis: &HumanPresenceAnalysis,
        energy_level: f32,
    ) -> Twist {
        let mut velocity = Twist::default();
        
        // Base speed on social impact and privacy concerns
        let base_speed = 0.5 * (1.0 - segment.social_impact) * (1.0 - segment.privacy_violation);
        
        // Adjust for energy level
        let energy_factor = if energy_level > 0.7 {
            1.0
        } else if energy_level > 0.4 {
            0.8
        } else {
            0.5
        };
        
        // Adjust for approach behavior
        let behavior_factor = match self.approach_behavior {
            ApproachBehavior::Avoidant => 0.7,
            ApproachBehavior::Neutral => 1.0,
            ApproachBehavior::Friendly => 1.2,
            ApproachBehavior::Assertive => 1.5,
        };
        
        velocity.linear.x = base_speed * energy_factor * behavior_factor * self.social_awareness_factor;
        
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
        
        command.linear.x = (self.current_velocity.linear.x + acceleration).max(0.0).min(1.0);
        
        // Update current velocity
        self.current_velocity = command.clone();
        self.last_command_time = Instant::now();
        
        command
    }
    
    fn update_safety_monitor(
        &mut self,
        command: &Twist,
        segment: &IndoorPathSegment,
        human_analysis: &HumanPresenceAnalysis,
    ) {
        // Update safety parameters
        self.safety_monitor.social_discomfort = segment.social_impact;
        self.safety_monitor.privacy_violation = segment.privacy_violation;
        
        // Calculate human proximity
        self.safety_monitor.human_proximity = self.calculate_human_proximity(human_analysis);
        
        // Check for emergency conditions
        if self.safety_monitor.human_proximity > self.safety_monitor.safety_thresholds.max_human_proximity ||
           self.safety_monitor.social_discomfort > self.safety_monitor.safety_thresholds.max_social_discomfort ||
           self.safety_monitor.privacy_violation > self.safety_monitor.safety_thresholds.max_privacy_violation {
            self.safety_monitor.emergency_stop_triggered = true;
        }
        
        // Check acceleration limits
        if command.linear.x.abs() > self.safety_monitor.safety_thresholds.max_acceleration {
            self.safety_monitor.emergency_stop_triggered = true;
        }
    }
    
    fn calculate_human_proximity(&self, human_analysis: &HumanPresenceAnalysis) -> f32 {
        // Calculate overall human proximity
        let mut proximity = 0.0;
        
        for human in &human_analysis.humans {
            let distance = (human.position.0.powi(2) + human.position.1.powi(2)).sqrt();
            proximity += (1.0 - distance.min(5.0) / 5.0); // Normalize to 0-1
        }
        
        (proximity / human_analysis.humans.len() as f32).min(1.0)
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
    
    pub fn adjust_social_awareness(&mut self, factor: f32) {
        self.social_awareness_factor = factor.max(0.1).min(2.0);
    }
    
    pub fn set_approach_behavior(&mut self, behavior: ApproachBehavior) {
        self.approach_behavior = behavior;
    }
    
    pub fn get_command_history(&self) -> &[(Twist, Instant)] {
        &self.command_history
    }
    
    pub fn generate_vocal_response(&self, human_analysis: &HumanPresenceAnalysis) -> Option<String> {
        // Generate appropriate vocal responses based on human presence
        if human_analysis.overall_activity_level > 0.7 {
            Some("Excuse me, I need to pass through".to_string())
        } else if self.safety_monitor.human_proximity > 0.6 {
            Some("Hello, I'm just passing by".to_string())
        } else {
            None
        }
    }
}