use r2r::QosProfile;

// Generic ROS2 Publisher wrapper
pub struct Publisher<T> {
    inner: r2r::Publisher<T>,
}

impl<T> Publisher<T>
where
    T: r2r::Message + 'static, // Message must implement ROS2 Message trait
{
    // Create a new publisher on the given topic with specified QoS
    pub fn new(node: &mut r2r::Node, topic: &str, qos: QosProfile) -> Result<Self, r2r::Error> {
        let publisher = node.create_publisher(topic, qos)?;
        Ok(Publisher { inner: publisher })
    }

    // Publish a message to the topic
    pub fn publish(&self, message: T) -> Result<(), r2r::Error> {
        self.inner.publish(message)
    }
}
