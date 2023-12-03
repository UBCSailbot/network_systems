# Launch Configs

`.yaml` files defining ROS node parameters for different network_systems modules.

## How to Run

To run with pure default defined in the code, run:

```
ros2 launch network_systems main_launch.py
```

To run with config files in this folder:

```
ros2 launch network_systems main_launch.py config:=<comma separated list of config files>
```

For example:

```
ros2 launch network_systems main_launch.py config:=default.yaml
```

launches network_systems with the parameters specified in `default.yaml`.

```
ros2 launch network_systems main_launch.py config:=default.yaml,example_enable.yaml
```

launches network_systems with the parameters specified in `default.yaml` *and* `example_enable.yaml`. Since
`example_enable.yaml` is specified after `default.yaml`, it overrides any duplicate parameters.
