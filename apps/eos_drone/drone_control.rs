use r2r::geometry_msgs::Twist;
use std::time::{Duration, Instant};

/// Flight control system for drones with 3D movement
pub struct DroneControl {
    current_velocity: Twist,
    max_acceleration: f32,
    max_deceleration: f32,
    weather_adaptation_factor: f32,
    safety_monitor: DroneSafetyMonitor,
    last_command_time: Instant,
    command_history: Vec<(Twist, Instant)>,
    hover_stability: f32,
}

pub struct DroneSafetyMonitor {
    emergency_land_triggered: bool,
    obstacle_proximity: f32,
    tilt_angle: f32,
    turbulence_level: f32,
    battery_health: f32,
    safety_thresholds: DroneSafetyThresholds,
}

pub struct DroneSafetyThresholds {
    pub max_tilt: f32,
    pub max_turbulence: f32,
    pub min_obstacle_distance: f32,
    pub min_battery_health: f32,
    pub max_altitude: f32,
}

impl DroneControl {
    pub fn new() -> Self {
        DroneControl {
            current_velocity: Twist::default(),
            max_acceleration: 0.3,
            max_deceleration: 0.4,
            weather_adaptation_factor: 1.0,
            safety_monitor: DroneSafetyMonitor {
                emergency_land_triggered: false,
                obstacle_proximity: 0.0,
                tilt_angle: 0.0,
                turbulence_level: 0.0,
                battery_health: 1.0,
                safety_thresholds: DroneSafetyThresholds {
                    max_tilt: 0.4,
                    max_turbulence: 0.7,
                    min_obstacle_distance: 1.0,
                    min_battery_health: 0.2,
                    max_altitude: 120.0, // meters
                },
            },
            last_command_time: Instant::now(),
            command_history: Vec::with_capacity(100),
            hover_stability: 0.9,
        }
    }
    
    pub fn execute_flight(
        &mut self,
        waypoint: &Waypoint,
        airspace_analysis: &DroneAirspaceAnalysis,
        weather_conditions: &WeatherConditions,
        energy_level: f32,
    ) -> Result<Twist, String> {
        // Check safety first
        if self.safety_monitor.emergency_land_triggered {
            return Err("Emergency landing active".to_string());
        }
        
        // Calculate optimal velocity for this flight segment
        let optimal_velocity = self.calculate_optimal_velocity(waypoint, airspace_analysis, weather_conditions, energy_level);
        
        // Smooth acceleration towards optimal velocity
        let command = self.smooth_acceleration(optimal_velocity);
        
        // Update safety monitor
        self.update_safety_monitor(&command, airspace_analysis, weather_conditions);
        
        // Record command
        self.record_command(command.clone());
        
        Ok(command)
    }
    
    fn calculate_optimal_velocity(
        &self,
        waypoint: &Waypoint,
        airspace: &DroneAirspaceAnalysis,
        weather: &WeatherConditions,
        energy_level: f32,
    ) -> Twist {
        let mut velocity = Twist::default();
        
        // Base speed on airspace conditions and weather
        let base_speed = waypoint.recommended_speed * (1.0 - airspace.turbulence_level) * (1.0 - weather.wind_speed * 0.1);
        
        // Adjust for energy level
        let energy_factor = if energy_level > 0.7 {
            1.0
        } else if energy_level > 0.4 {
            0.8
        } else if energy_level > 0.2 {
            0.5
        } else {
            0.3
        };
        
        velocity.linear.x = base_speed * energy_factor * self.weather_adaptation_factor;
        velocity.linear.z = self.calculate_vertical_velocity(waypoint, airspace, weather);
        
        // Apply wind compensation
        velocity.linear.x += waypoint.wind_compensation.0;
        velocity.linear.y += waypoint.wind_compensation.1;
        velocity.linear.z += waypoint.wind_compensation.2;
        
        velocity
    }
    
    fn calculate_vertical_velocity(
        &self,
        waypoint: &Waypoint,
        airspace: &DroneAirspaceAnalysis,
        weather: &WeatherConditions,
    ) -> f32 {
        // Calculate vertical velocity based on altitude difference
        let current_altitude = 0.0; // Would come from localizer
        let target_altitude = waypoint.position.pose.position.z;
        let altitude_diff = target_altitude - current_altitude;
        
        // Base vertical speed with weather adjustment
        let base_speed = if altitude_diff > 0.0 { 0.5 } else { -0.3 };
        base_speed * (1.0 - weather.wind_speed * 0.05)
    }
    
    fn smooth_acceleration(&mut self, target_velocity: Twist) -> Twist {
        let time_since_last = self.last_command_time.elapsed().as_secs_f32();
        let mut command = self.current_velocity.clone();
        
        // Calculate acceleration needed for each axis
        let accel_x = self.calculate_axis_acceleration(
            target_velocity.linear.x,
            self.current_velocity.linear.x,
            time_since_last
        );
        
        let accel_y = self.calculate_axis_acceleration(
            target_velocity.linear.y,
            self.current_velocity.linear.y,
            time_since_last
        );
        
        let accel_z = self.calculate_axis_acceleration(
            target_velocity.linear.z,
            self.current_velocity.linear.z,
            time_since_last
        );
        
        command.linear.x = (self.current_velocity.linear.x + accel_x).max(-5.0).min(5.0);
        command.linear.y = (self.current_velocity.linear.y + accel_y).max(-5.0).min(5.0);
        command.linear.z = (self.current_velocity.linear.z + accel_z).max(-2.0).min(2.0);
        
        // Update current velocity
        self.current_velocity = command.clone();
        self.last_command_time = Instant::now();
        
        command
    }
    
    fn calculate_axis_acceleration(&self, target: f32, current: f32, time_delta: f32) -> f32 {
        let diff = target - current;
        if diff > 0.0 {
            diff.min(self.max_acceleration * time_delta)
        } else {
            diff.max(-self.max_deceleration * time_delta)
        }
    }
    
    fn update_safety_monitor(
        &mut self,
        command: &Twist,
        airspace: &DroneAirspaceAnalysis,
        weather: &WeatherConditions,
    ) {
        // Update safety parameters
        self.safety_monitor.turbulence_level = airspace.turbulence_level;
        
        // Check for emergency conditions
        if self.safety_monitor.tilt_angle > self.safety_monitor.safety_thresholds.max_tilt ||
           self.safety_monitor.turbulence_level > self.safety_monitor.safety_thresholds.max_turbulence ||
           self.safety_monitor.obstacle_proximity < self.safety_monitor.safety_thresholds.min_obstacle_distance ||
           self.safety_monitor.battery_health < self.safety_monitor.safety_thresholds.min_battery_health {
            self.safety_monitor.emergency_land_triggered = true;
        }
        
        // Check altitude limit
        let current_altitude = 0.0; // Would come from localizer
        if current_altitude > self.safety_monitor.safety_thresholds.max_altitude {
            self.safety_monitor.emergency_land_triggered = true;
        }
    }
    
    fn record_command(&mut self, command: Twist) {
        if self.command_history.len() >= 100 {
            self.command_history.remove(0);
        }
        self.command_history.push((command, Instant::now()));
    }
    
    pub fn emergency_land(&mut self) -> Twist {
        self.safety_monitor.emergency_land_triggered = true;
        
        // Generate landing command with safe descent
        let mut land_cmd = Twist::default();
        land_cmd.linear.z = -0.3; // Slow descent
        land_cmd.linear.x = 0.0;
        land_cmd.linear.y = 0.0;
        
        // Record emergency landing
        self.record_command(land_cmd.clone());
        
        land_cmd
    }
    
    pub fn reset_safety(&mut self) {
        self.safety_monitor.emergency_land_triggered = false;
    }
    
    pub fn adjust_weather_adaptation(&mut self, factor: f32) {
        self.weather_adaptation_factor = factor.max(0.1).min(2.0);
    }
    
    pub fn maintain_hover(&mut self) -> Twist {
        // Generate hover command with stability adjustments
        let mut hover_cmd = Twist::default();
        hover_cmd.linear.z = 0.1 * (1.0 - self.hover_stability); // Small adjustment
        
        hover_cmd
    }
    
    pub fn get_command_history(&self) -> &[(Twist, Instant)] {
        &self.command_history
    }
}