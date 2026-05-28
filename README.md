# robot_impedance_analyzer

This package provides robot impedance analysis tools, such as kinematic reference generation based on the [Kinematic Pose](https://github.com/qleonardolp/kinematic_pose_msgs) message, publishing a dynamic pose to be tracked by impedance controllers. The pose, with twist and twsit derivative, can be seen as equilibrium point for the impedance dynamics, or, in a classical sense, the controller reference.

The node parameters are handled using the `generate_parameter_library`.

```console
ros2 run robot_impedance_analyzer kinematic_reference --ros-args --params-file src/robot_impedance_analyzer/config/parameters.yaml
```

After this command the lifecycle node is _unconfigured_.
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

## Impedance Identification

```console
ros2 launch robot_impedance_analyzer identify.launch.py
```

Then:

```console
ros2 lifecycle set /identification activate
```
