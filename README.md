# dbw_ros

## Topics

* dbw_enabled (std_msgs/msg/Bool) QoS(1,TransientLocal) - 

* vin (std_msgs/msg/String) QoS(1,TransientLocal) - 

## Quality of Serice (QoS)
Quality of service definition is as follows. This section refers to how to relates to the above Topic documentation. See the [ROS2 QoS Documentation](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html) for more details on QoS.

A QoS definition for a topic will have the following format: `QoS(History,Depth,Reliability,Durability)`. Any value may be omitted without losing clarity, including the entire QoS definition itself. Any omitted values are assumed to be the default.

### History

* KeepLast - only store up to N samples, configurable via the queue depth option. **Default**.

* KeepAll - store all samples, subject to the configured resource limits of the DDS vendor. With this option, Queue Size is ignored.

### Depth

* Queue Size - Depicted in the definition as an integer. Only honored if used together with keep last. **Default of 10**

### Reliability

* BestEffort - Attempt to deliver samples, but may lose them if the network is not robust.

* Reliable - Guarantee that samples are delivered, may retry multiple times. **Default**

### Durability

* TransientLocal - The publisher becomes responsible for persisting samples for late-joining subscriptions. This will only occur if both the publisher and subscriber use this Durablity.

* Volatile - No attempt is made to persist samples. **Default**

