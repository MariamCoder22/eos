//! Neural network integration for Eos OS
//!
//! This module provides interfaces to spiking neural networks (SNN)
//! for sensor processing and decision making.
pub mod snn;
pub mod config;

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// SNN engine for processing sensor data
pub struct SNNEngine {
    config: NeuralConfig,
    model: Option<NeuralModel>,
    input_buffer: VecDeque<Vec<f32>>,
    output_buffer: VecDeque<Vec<f32>>,
    is_initialized: bool,
}

/// Neural network configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NeuralConfig {
    /// Input size for the neural network
    pub input_size: usize,
    /// Output size from the neural network
    pub output_size: usize,
    /// Number of hidden layers
    pub hidden_layers: usize,
    /// Neurons per hidden layer
    pub hidden_neurons: usize,
    /// Learning rate
    pub learning_rate: f32,
    /// Spike threshold
    pub spike_threshold: f32,
    /// Simulation time steps
    pub time_steps: usize,
}

/// Neural network model structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NeuralModel {
    /// Model weights
    weights: Vec<Vec<f32>>,
    /// Neuron states
    states: Vec<f32>,
    /// Neuron thresholds
    thresholds: Vec<f32>,
    /// Model metadata
    metadata: ModelMetadata,
}

/// Model metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
struct ModelMetadata {
    /// Model name
    name: String,
    /// Model version
    version: String,
    /// Training dataset
    trained_on: String,
    /// Performance metrics
    accuracy: f32,
}

/// Neural network status
#[derive(Debug, Clone)]
pub struct NeuralStatus {
    /// Whether the model is loaded
    pub model_loaded: bool,
    /// Input buffer size
    pub input_buffer_size: usize,
    /// Output buffer size
    pub output_buffer_size: usize,
    /// Processing latency in milliseconds
    pub processing_latency: f32,
    /// Model accuracy if available
    pub model_accuracy: Option<f32>,
}

impl Default for NeuralConfig {
    fn default() -> Self {
        NeuralConfig {
            input_size: 100,
            output_size: 10,
            hidden_layers: 2,
            hidden_neurons: 64,
            learning_rate: 0.01,
            spike_threshold: 0.5,
            time_steps: 10,
        }
    }
}

impl SNNEngine {
    /// Create a new SNN engine
    pub fn new(config: &NeuralConfig) -> Result<Self, NeuralError> {
        Ok(SNNEngine {
            config: config.clone(),
            model: None,
            input_buffer: VecDeque::with_capacity(100),
            output_buffer: VecDeque::with_capacity(100),
            is_initialized: false,
        })
    }
    
    /// Initialize the neural engine
    pub fn initialize(&mut self) -> Result<(), NeuralError> {
        log::info!("Initializing neural engine...");
        
        // Create a default model if none loaded
        if self.model.is_none() {
            self.model = Some(self.create_default_model());
            log::info!("Created default neural model");
        }
        
        self.is_initialized = true;
        log::info!("Neural engine initialized successfully");
        
        Ok(())
    }
    
    /// Load a pre-trained model
    pub fn load_model(&mut self, path: &str) -> Result<(), NeuralError> {
        log::info!("Loading neural model from: {}", path);
        
        let model_data = std::fs::read_to_string(path)
            .map_err(|e| NeuralError::LoadError(e.to_string()))?;
            
        let model: NeuralModel = serde_json::from_str(&model_data)
            .map_err(|e| NeuralError::ParseError(e.to_string()))?;
            
        self.model = Some(model);
        log::info!("Neural model loaded successfully");
        
        Ok(())
    }
    
    /// Save the current model
    pub fn save_model(&self, path: &str) -> Result<(), NeuralError> {
        if let Some(model) = &self.model {
            let model_data = serde_json::to_string_pretty(model)
                .map_err(|e| NeuralError::SaveError(e.to_string()))?;
                
            std::fs::write(path, model_data)
                .map_err(|e| NeuralError::SaveError(e.to_string()))?;
                
            log::info!("Neural model saved to: {}", path);
            Ok(())
        } else {
            Err(NeuralError::NoModelError)
        }
    }
    
    /// Process sensor data through the neural network
    pub fn process(&mut self, sensor_data: &super::ros_interface::SensorData) -> Result<Vec<f32>, NeuralError> {
        if !self.is_initialized {
            return Err(NeuralError::NotInitialized);
        }
        
        // Convert sensor data to neural network input
        let input = self.preprocess_sensor_data(sensor_data);
        
        // Add to input buffer
        self.input_buffer.push_back(input.clone());
        if self.input_buffer.len() > 100 {
            self.input_buffer.pop_front();
        }
        
        // Process through neural network
        let start_time = std::time::Instant::now();
        let output = self.process_input(&input)?;
        let processing_time = start_time.elapsed();
        
        // Add to output buffer
        self.output_buffer.push_back(output.clone());
        if self.output_buffer.len() > 100 {
            self.output_buffer.pop_front();
        }
        
        log::debug!("Neural processing time: {:?}", processing_time);
        
        Ok(output)
    }
    
    /// Get current neural engine status
    pub fn get_status(&self) -> NeuralStatus {
        NeuralStatus {
            model_loaded: self.model.is_some(),
            input_buffer_size: self.input_buffer.len(),
            output_buffer_size: self.output_buffer.len(),
            processing_latency: 0.0, // Would be calculated from actual timing
            model_accuracy: self.model.as_ref().map(|m| m.metadata.accuracy),
        }
    }
    
    /// Preprocess sensor data for neural network input
    fn preprocess_sensor_data(&self, sensor_data: &super::ros_interface::SensorData) -> Vec<f32> {
        // Simple preprocessing - would be more complex in production
        let mut input = Vec::with_capacity(self.config.input_size);
        
        // Add laser scan data
        if sensor_data.laser_scan.ranges.len() > 0 {
            let scan_data = &sensor_data.laser_scan.ranges;
            for i in 0..self.config.input_size.min(scan_data.len()) {
                input.push(scan_data[i] as f32);
            }
        }
        
        // Pad with zeros if needed
        while input.len() < self.config.input_size {
            input.push(0.0);
        }
        
        input
    }
    
    /// Process input through the neural network
    fn process_input(&self, input: &[f32]) -> Result<Vec<f32>, NeuralError> {
        if let Some(model) = &self.model {
            // Simple feedforward simulation - would use actual SNN in production
            let mut output = vec![0.0; self.config.output_size];
            
            for i in 0..self.config.output_size {
                for j in 0..input.len().min(model.weights.len()) {
                    output[i] += input[j] * model.weights[j][i];
                }
                // Simple activation (would be spike-based in real SNN)
                output[i] = if output[i] > self.config.spike_threshold {
                    1.0
                } else {
                    0.0
                };
            }
            
            Ok(output)
        } else {
            Err(NeuralError::NoModelError)
        }
    }
    
    /// Create a default model with random weights
    fn create_default_model(&self) -> NeuralModel {
        let mut weights = Vec::with_capacity(self.config.input_size);
        
        for _ in 0..self.config.input_size {
            let mut neuron_weights = Vec::with_capacity(self.config.output_size);
            for _ in 0..self.config.output_size {
                neuron_weights.push(rand::random::<f32>() * 2.0 - 1.0);
            }
            weights.push(neuron_weights);
        }
        
        NeuralModel {
            weights,
            states: vec![0.0; self.config.input_size],
            thresholds: vec![self.config.spike_threshold; self.config.input_size],
            metadata: ModelMetadata {
                name: "default_model".to_string(),
                version: "1.0".to_string(),
                trained_on: "random_weights".to_string(),
                accuracy: 0.0,
            },
        }
    }
}

/// Neural network error types
#[derive(Debug)]
pub enum NeuralError {
    /// Engine not initialized
    NotInitialized,
    /// No model loaded
    NoModelError,
    /// Model load error
    LoadError(String),
    /// Model save error
    SaveError(String),
    /// Model parse error
    ParseError(String),
    /// Processing error
    ProcessingError(String),
}

impl std::fmt::Display for NeuralError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            NeuralError::NotInitialized => write!(f, "Neural engine not initialized"),
            NeuralError::NoModelError => write!(f, "No neural model loaded"),
            NeuralError::LoadError(msg) => write!(f, "Model load error: {}", msg),
            NeuralError::SaveError(msg) => write!(f, "Model save error: {}", msg),
            NeuralError::ParseError(msg) => write!(f, "Model parse error: {}", msg),
            NeuralError::ProcessingError(msg) => write!(f, "Processing error: {}", msg),
        }
    }
}

impl std::error::Error for NeuralError {}

