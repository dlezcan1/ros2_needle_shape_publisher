# ROS 2 Needle Shape Publisher
Author: Dimitri Lezcano

This is a repository for publishing the needle shape for the Sensorized Needle Robot project in collaboration with Brigham Women's Hospital.

### Needle Frame Orientation
The needle frame's origin is set to be the end of the insertion needle guide. The orientation of the needle frame is given by

<img src="info/needle_frame.png" alt="Needle Frame Orientation" style="float: center"/>

### Dependencies
* MATLAB R2021a
* ROS Toolbox
* Optimization Toolbox

## Description
NeedleShapePublisher class 
* node of the needle shape publisher

### Subscribers 
* /stage/state/needle_pose: "geometry_msgs/msg/PoseStamped" of the needle pose in the jig
* /needle/sensors/processed: "std_msgs/msg/Float64MultiArray" of the sensorized needle's calibrated signal shifts
* /subject/state/entry_point: "std_msgs/msg/PointStamped" the current entry point of the needle in the patient

### Publishers
* /needle/shape/current: "geometry_msgs/msg/PoseArray" the pose array of the needle shape in 0.5 mm increments
* /needle/shape/predicted: "geometry_msgs/msg/PoseArray" the predicted pose array of the needle shape in 0.5 mm increments
