import numpy as np
import json

class SNNModel:
    def __init__(self, config_path):
        # Load configuration file (JSON)
        with open(config_path, 'r') as f:
            self.config = json.load(f)

        # Initialize random weights for neurons
        self.weights = np.random.randn(
            self.config['neuron_count'],
            self.config['neuron_count']
        )
        
    def process(self, sensor_data):
        # Simple simulation of SNN: weighted sum and thresholding
        
        # Convert input to numpy array
        input_data = np.array(sensor_data)

        # Adjust input size to match neuron count
        if len(input_data) < self.config['neuron_count']:
            # Pad with zeros if input is too short
            input_data = np.pad(
                input_data,
                (0, self.config['neuron_count'] - len(input_data))
            )
        else:
            # Truncate if input is too long
            input_data = input_data[:self.config['neuron_count']]
        
        # Simulate network activity with tanh activation
        output = np.tanh(np.dot(self.weights, input_data))
        
        return output.tolist()


if __name__ == "__main__":
    # For quick testing
    model = SNNModel("config/snn_config.json")
    result = model.process([0.1, 0.2, 0.3])
    print(result)
