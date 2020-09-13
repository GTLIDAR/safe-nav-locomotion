# safe-nav-locomotion

This repo contains the code used for implementation in our published work:-
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


The Motion_Planner directory contains the code nacessary to generate the center of mass, foot trajectories as well as foot placement location using phase-space planning. It also containts a Drake visualization code of the Cassie bipedal robot following the generated trajectories in the proposed environment.
