Turtle Bot Examples
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


Basic Turtle Bot Simulation
---------------------
None!!!

Turtle Bot Position Publisher
---------------------

This will publish position statuses to collaboration_station/simulation_visualizer
Check out how to run simulation_visualizer in the file: 
  drake/examples/collaboration_station/simulation_visualizer.cc

```
bazel run //examples/mobile_robots/turtle_bot:turtle_position_publisher
```


