Collaboration Examples
==========================

There are a number of examples contained in these directories.

The following instructions assume Drake was
[built using bazel](https://drake.mit.edu/bazel.html?highlight=bazel).

Prerequisites
-------------

Ensure that you have installed the drake visualizer with
```
bazel build //tools:drake_visualizer
```

All instructions assume that you are launching from the `drake`
workspace directory.
```
cd drake
```


Basic Collaboration Simulation
---------------------
!!! Change the config path to drake !!!
(This step assumes you have vscode installed)

```
code ./examples/collaboration/config.json
```

Change the path

"path_to_drake":"/home/type_your_computer_name/",


Chose your robots

In the code, find "#define robot_name boolean_value"
(Example would be #define USE_IIWA true)

Set whichever robot you want in the simulation.


Save and open back terminal

Launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

Launch the collaboration simulation
```
bazel run //examples/collaboration_station:simulation_visualizer
```

Open a new terminal and launch the cassie position publisher
```
bazel run //examples/cassie:cassie_position_publisher
```

Open yet another new terminal and launch the turtle position publisher
```
bazel run //examples/mobile_robots/turtle_bot:turtle_position_publisher
```

Note: The position publishers has to run right after the simulation started.
Otherwise the publishers won't publish because time ran out. 
I will be fixing this next time, but for now, speed is the key!

