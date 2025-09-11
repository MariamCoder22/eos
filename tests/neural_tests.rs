#[cfg(test)]
mod tests {
    use super::*;

    // Unit test for the SNNEngine
    #[test]
    fn test_snn_engine() {
        // Use default config
        let config = SNNConfig::default();

        // Create a new engine
        let mut engine = SNNEngine::new(config);

        // Example sensor input
        let sensor_data = vec![0.1, 0.2, 0.3];

        // Process the sensor data through the engine
        let result = engine.process_sensor_data(&sensor_data).unwrap();

        // Verify output has expected size (2 outputs, e.g. linear + angular velocity)
        assert_eq!(result.len(), 2);
    }
}
