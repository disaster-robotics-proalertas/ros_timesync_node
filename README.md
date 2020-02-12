# ROS timesync status node
ROS node to compare and report (as diagnostics) the status of time synchonization in a ROS environment.

## Parameters

* topic: Topic with time reference from master
* mode: Defines node operation. Server: Node publishes its own walltime on topic indicated by the parameter above; Client: Node gets time reference from topic and compares it to its own time, up to a certain tolerance.
* tolerance: Threshold where clocks are considered to be synchronized
