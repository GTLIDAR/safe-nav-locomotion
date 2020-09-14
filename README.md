
# Task Planner

# Motion Planner

The Motion_Planner directory contains the code nacessary to generate the center of mass, foot trajectories as well as foot placement location using phase-space planning. It also containts a Drake visualization code of the Cassie bipedal robot following the generated trajectories in the proposed environment.

## Drake Phase-Space Planning and Visualization 

The code is based on Drake (Please see the [Drake Documentation](https://drake.mit.edu) for more
information). Here we include the source code of Drake and our own addition for phase-space planning in the [safe-nav-loco](motion_planner/drake/safe-nav-loco/) folder.

The code is run on Ubuntu 16.04.

## Initializing 

In a terminal go to the directory where you want to clone safe-nav-locomotion repo.
run 
`git clone https://github.com/GTLIDAR/safe-nav-locomotion.git`

### Building Drake
Make sure you have the required dependencies for Drake. 
Drake installation steps can be found [here](https://drake.mit.edu/installation.html).

### Local adjustments 

in [motion_planner/drake/safe-nav-loco/src/simulate_psp.cc](motion_planner/drake/safe-nav-loco/src/simulate_psp.cc) and [motion_planner/drake/safe-nav-loco/src/run_cassie_follow.cc](motion_planner/drake/safe-nav-loco/src/run_cassie_follow.cc) adjust the path in `file_name = "path/drake/safe-nav-loco/vis/..."` to match the path to the drake directory on your local machine. 

## Running the code
### Setting up action.json file from task_planner
1- Once an action.json file is generated from task_planner, copy and past the file into [motion_planner/drake/safe-nav-loco/vis/](motion_planner/drake/safe-nav-loco/vis/).
Note:[actions_CDC_Subs.json](motion_planner/drake/safe-nav-loco/vis/actions_CDC_Sub.json) is the action file used in our publication

2- In [motion_planner/drake/safe-nav-loco/src/simulate_psp.cc](motion_planner/drake/safe-nav-loco/src/simulate_psp.cc) adjust the path in 
`BeliefIOParser parser("path/drake/safe-nav-loco/vis/actions_CDC_Sub.json");` and choose the desired action file.

### Phase-Space Planning and trajectory generation 
1- Open terminal and go to the drake folder `cd path/drake/`

2- run `CC=clang-6.0 CXX=clang++-6.0 bazel run CDC:simulate_psp`
This will generate the trajectory .txt files.

### Drake Visualization 
<img src="https://i.imgur.com/taoI3AF.gif" />

Make sure that the trajectories are generated beforehand as shown in the previous section.

1- Open termminal and run the command to open Drake visualizer
```
cd path/drake/
bazel-bin/tools/drake_visualizer
```
2- Open another terminal and run the commend to open the drake lcm spy
```
cd path/drake/
bazel-bin/lcmtypes/drake-lcm-spy
```
3- Open a third terminal and run the command to simulate the Cassie robot in drake visualzier within the environment
```
cd path/drake/
CC=clang-6.0 CXX=clang++-6.0 bazel run CDC:run_cassie_follow
```

### Py.plot Visualization 
<img src="https://i.imgur.com/jJ5KXOj.png" />

1- Open terminal and run the command to visualize the center of mass and foot trajectories as well as high-level waypoints, and foot stance locations.

```
cd path/drake/safe-nav-loco/vis/
python vis_psp.py 
```


# Project and Related Publications 

This work is a part of our ongoing work on robust and reactive decision-making and AI planning of collaborative and agile robots in complex environments. More information and related publications can be found [here](http://lab-idar.gatech.edu/robust-and-reactive-decision-making-and-ai-planning-of-collaborative-and-agile-robots-in-complex-environments/)


This repo contains the code used for implementation in our [published work](https://arxiv.org/abs/2009.05168):-
```
@article{warnketowards,
  title={Towards Safe Locomotion Navigation in 
  Partially Observable Environments with Uneven Terrain},
  author={Warnke, Jonas and Shamsah, Abdulaziz 
  and Li, Yingke and Zhao, Ye}
  journal   = {IEEE Conference on Decision and Control},
  year      = {2020},
}
```


# About
