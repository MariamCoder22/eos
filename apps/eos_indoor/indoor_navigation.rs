use crate::core::{Localizer, SpatialMemory};
use r2r::geometry_msgs::{PoseStamped, Twist};
use std::collections::HashMap;

/// Indoor navigation with social awareness and human interaction
pub struct IndoorNavigation {
    localizer: Localizer,
    spatial_memory: SpatialMemory,
    room_maps: HashMap<String, RoomMap>,
    current_path: Option<IndoorPath>,
    social_awareness_factor: f32,
    human_interaction_mode: HumanInteractionMode,
}

pub struct RoomMap {
    pub name: String,
    pub dimensions: (f32, f32, f32), // width, depth, height
    pub furniture: Vec<Furniture>,
    pub walkable_areas: Vec<(f32, f32, f32, f32)>, // x1, y1, x2, y2
    pub social_zones: Vec<SocialZone>,
}

pub struct Furniture {
    pub position: (f32, f32),
    pub dimensions: (f32, f32, f32),
    pub type_name: String,
}

pub struct SocialZone {
    pub position: (f32, f32),
    pub radius: f32,
    pub zone_type: SocialZoneType,
    pub privacy_level: f32,
}

pub enum SocialZoneType {
    Conversation,
    Work,
    Relaxation,
    Private,
}

pub struct IndoorPath {
    pub segments: Vec<IndoorPathSegment>,
    pub total_energy_estimate: f32,
    pub social_acceptability: f32,
    pub privacy_respect: f32,
}

pub struct IndoorPathSegment {
    pub start: PoseStamped,
    pub end: PoseStamped,
    pub room_name: String,
    pub social_impact: f32,
    pub energy_estimate: f32,
    pub privacy_violation: f32,
}

pub enum HumanInteractionMode {
    Passive,    // Avoid humans
    Assistive,  // Help humans
    Interactive, // Engage with humans
    Emergency,  // Override social rules
}

impl IndoorNavigation {
    pub fn new(localizer: Localizer, spatial_memory: SpatialMemory) -> Self {
        IndoorNavigation {
            localizer,
            spatial_memory,
            room_maps: HashMap::new(),
            current_path: None,
            social_awareness_factor: 0.8,
            human_interaction_mode: HumanInteractionMode::Passive,
        }
    }
    
    pub fn load_room_maps(&mut self, path: &str) -> Result<(), String> {
        let data = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read room maps: {}", e))?;
            
        let maps: Vec<RoomMap> = serde_json::from_str(&data)
            .map_err(|e| format!("Failed to parse room maps: {}", e))?;
            
        for room_map in maps {
            self.room_maps.insert(room_map.name.clone(), room_map);
        }
        
        Ok(())
    }
    
    pub fn plan_indoor_path(
        &mut self,
        goal: PoseStamped,
        indoor_analysis: &IndoorEnvironmentAnalysis,
        human_analysis: &HumanPresenceAnalysis,
        energy_level: f32,
    ) -> Result<IndoorPath, String> {
        let current_pose = self.localizer.get_current_pose();
        let mut path = IndoorPath {
            segments: Vec::new(),
            total_energy_estimate: 0.0,
            social_acceptability: 1.0,
            privacy_respect: 1.0,
        };
        
        // Check if goal is in a restricted area
        if let Some(room) = self.get_room_for_position(&goal) {
            if self.is_restricted_area(room, human_analysis) {
                return Err("Goal is in a restricted area".to_string());
            }
        }
        
        // Plan path considering social rules and human presence
        let segments = self.generate_path_segments(current_pose, &goal, indoor_analysis, human_analysis);
        
        for segment in segments {
            let energy = self.calculate_energy_cost(&segment, indoor_analysis);
            let social_impact = self.calculate_social_impact(&segment, human_analysis);
            let privacy_violation = self.calculate_privacy_violation(&segment, human_analysis);
            
            path.segments.push(IndoorPathSegment {
                start: segment.start,
                end: segment.end,
                room_name: segment.room_name,
                social_impact,
                energy_estimate: energy,
                privacy_violation,
            });
            
            path.total_energy_estimate += energy;
            path.social_acceptability *= 1.0 - (social_impact * 0.1);
            path.privacy_respect *= 1.0 - (privacy_violation * 0.2);
        }
        
        // Check energy constraints
        if path.total_energy_estimate > energy_level * 0.8 {
            return Err("Insufficient energy for indoor path".to_string());
        }
        
        // Check social acceptability
        if path.social_acceptability < 0.5 {
            return Err("Path is not socially acceptable".to_string());
        }
        
        self.current_path = Some(path.clone());
        Ok(path)
    }
    
    fn get_room_for_position(&self, position: &PoseStamped) -> Option<&RoomMap> {
        // Find which room this position is in
        for room in self.room_maps.values() {
            if self.is_position_in_room(position, room) {
                return Some(room);
            }
        }
        None
    }
    
    fn is_position_in_room(&self, position: &PoseStamped, room: &RoomMap) -> bool {
        // Check if position is within room boundaries
        let x = position.pose.position.x;
        let y = position.pose.position.y;
        
        x >= 0.0 && x <= room.dimensions.0 && y >= 0.0 && y <= room.dimensions.1
    }
    
    fn is_restricted_area(&self, room: &RoomMap, human_analysis: &HumanPresenceAnalysis) -> bool {
        // Check if this area should be restricted based on human presence
        for zone in &room.social_zones {
            if zone.zone_type == SocialZoneType::Private && zone.privacy_level > 0.7 {
                // Check if humans are present in this private zone
                for human in &human_analysis.humans {
                    let distance = ((human.position.0 - zone.position.0).powi(2) + 
                                  (human.position.1 - zone.position.1).powi(2)).sqrt();
                    
                    if distance < zone.radius {
                        return true;
                    }
                }
            }
        }
        
        false
    }
    
    fn generate_path_segments(
        &self,
        start: PoseStamped,
        end: PoseStamped,
        indoor_analysis: &IndoorEnvironmentAnalysis,
        human_analysis: &HumanPresenceAnalysis,
    ) -> Vec<IndoorPathSegment> {
        // Complex indoor path generation considering social rules
        let mut segments = Vec::new();
        
        // Simple straight-line path for MVP
        segments.push(IndoorPathSegment {
            start: start.clone(),
            end: end.clone(),
            room_name: "unknown".to_string(),
            social_impact: 0.0,
            energy_estimate: 0.1,
            privacy_violation: 0.0,
        });
        
        segments
    }
    
    fn calculate_energy_cost(&self, segment: &IndoorPathSegment, indoor_analysis: &IndoorEnvironmentAnalysis) -> f32 {
        // Energy cost calculation based on floor type and obstacles
        let base_energy = 0.1; // Energy per meter
        let floor_penalty = match indoor_analysis.floor_type.as_str() {
            "carpet" => 0.2,
            "tile" => 0.0,
            "wood" => 0.1,
            "rug" => 0.3,
            _ => 0.0,
        };
        
        base_energy + floor_penalty
    }
    
    fn calculate_social_impact(&self, segment: &IndoorPathSegment, human_analysis: &HumanPresenceAnalysis) -> f32 {
        // Calculate social impact of moving through this segment
        let mut impact = 0.0;
        
        for human in &human_analysis.humans {
            let distance = self.distance_to_segment(&human.position, segment);
            if distance < 2.0 { // Close to human
                impact += (2.0 - distance) * 0.5;
            }
        }
        
        impact.min(1.0)
    }
    
    fn calculate_privacy_violation(&self, segment: &IndoorPathSegment, human_analysis: &HumanPresenceAnalysis) -> f32 {
        // Calculate privacy violation of moving through this segment
        let mut violation = 0.0;
        
        for human in &human_analysis.humans {
            if human.activity == "private" {
                let distance = self.distance_to_segment(&human.position, segment);
                if distance < 1.5 { // Very close to human in private activity
                    violation += (1.5 - distance) * 0.7;
                }
            }
        }
        
        violation.min(1.0)
    }
    
    fn distance_to_segment(&self, point: &(f32, f32), segment: &IndoorPathSegment) -> f32 {
        // Calculate distance from point to path segment
        let x1 = segment.start.pose.position.x;
        let y1 = segment.start.pose.position.y;
        let x2 = segment.end.pose.position.x;
        let y2 = segment.end.pose.position.y;
        let x0 = point.0;
        let y0 = point.1;
        
        // Simplified distance calculation
        ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt().min(
        ((x2 - x0).powi(2) + (y2 - y0).powi(2)).sqrt())
    }
    
    pub fn adjust_for_human_changes(
        &mut self,
        new_human_analysis: &HumanPresenceAnalysis,
        energy_level: f32,
    ) -> Result<(), String> {
        if let Some(path) = &mut self.current_path {
            // Recalculate social impact and privacy violation
            for segment in &mut path.segments {
                segment.social_impact = self.calculate_social_impact(segment, new_human_analysis);
                segment.privacy_violation = self.calculate_privacy_violation(segment, new_human_analysis);
            }
            
            // Recalculate totals
            path.social_acceptability = path.segments.iter().map(|s| 1.0 - (s.social_impact * 0.1)).product();
            path.privacy_respect = path.segments.iter().map(|s| 1.0 - (s.privacy_violation * 0.2)).product();
            
            // Check if still acceptable
            if path.social_acceptability < 0.5 {
                return Err("Path became socially unacceptable after human changes".to_string());
            }
            
            Ok(())
        } else {
            Err("No current path to adjust".to_string())
        }
    }
    
    pub fn set_human_interaction_mode(&mut self, mode: HumanInteractionMode) {
        self.human_interaction_mode = mode;
    }
    
    pub fn get_navigation_commands(&self) -> Option<Twist> {
        // Generate socially-aware movement commands
        if let Some(path) = &self.current_path {
            let mut cmd = Twist::default();
            cmd.linear.x = 0.3; // Slow indoor speed
            Some(cmd)
        } else {
            None
        }
    }
}

// Additional structs for indoor navigation
pub struct IndoorEnvironmentAnalysis {
    pub floor_type: String,
    pub obstacle_density: f32,
    room_type: String,
    pub lighting_level: f32,
}

pub struct HumanPresenceAnalysis {
    pub humans: Vec<Human>,
    pub overall_activity_level: f32,
}

pub struct Human {
    pub position: (f32, f32),
    pub activity: String,
    pub attention: f32,
    pub group_size: u8,
}