#!/bin/bash

gnome-terminal --tab --title=newTab \\
               -- bash -c "bazel run -j 2 //examples/collaboration_station:simulation_visualizer ;bash"
gnome-terminal --tab --title=newTab \\
               -- bash -c "bazel run -j 2 //examples/collaboration_station/kuka:move_iiwa_ee ;bash"
gnome-terminal --tab --title=newTab \\
               -- bash -c "bazel run -j 2 //examples/collaboration_station/kuka:kuka_position_publisher ;bash"
