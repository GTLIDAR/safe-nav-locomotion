Cassie Examples
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


Basic Cassie Simulation
---------------------
!!! Change the config path to drake !!!
(This step assumes you have vscode installed)

```
code ./examples/cassie/config.json
```

Change the path

"path_to_drake":"/home/type_your_computer_name/",

Save and open back terminal

Launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

Launch the cassie simulation
```
bazel run //examples/cassie:run_cassie_follow
```

Cassie and TurtleBot Simulation
---------------------
!!! Change the config path to drake !!!
(This step assumes you have vscode installed)

```
code ./examples/cassie/config.json
```

Launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

Launch the cassie_turtle simulation
```
bazel run //examples/cassie:run_cassie_follow_turtle
```