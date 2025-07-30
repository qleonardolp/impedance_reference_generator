# impedance_reference_generator

This package provides kinematic reference generation nodes based on the [Kinematic Pose](https://github.com/qleonardolp/kinematic_pose_msgs) message, for impedance controllers pose tracking.

The node parameters are handled using the `generate_parameter_library`. Set the parameters in [config/parameters.yaml](config/parameters.yaml) and run the node:

```console
ros2 run impedance_reference_generator kinematic_reference --ros-args --params-file src/impedance_reference_generator/config/parameters.yaml
```

Then, transition the lifecycle to activate the publication:

```console
ros2 lifecycle set /kinematic_reference configure

ros2 lifecycle set /kinematic_reference activate
```


You can change the signal by setting the parameters while the node is inactive:

```console
ros2 lifecycle set /kinematic_reference deactivate

ros2 param set /kinematic_reference amplitude 0.100

ros2 param set /kinematic_reference rate 250

ros2 lifecycle set /kinematic_reference activate
```