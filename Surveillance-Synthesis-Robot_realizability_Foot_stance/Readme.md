# Synthesis of Surveillance Strategies for Autonomous Agents

## Suda Bharadwaj, Rayna Dimitrova, Jesse Quattrociocchi, Ufuk Topcu

### ROS+Gazebo simulations
First we consider the safety surveillance task $\LTLglobally p_1$. The quadcopter is never allowed to lose sight of the target. 

{% include youtubePlayer.html %}

Note that the quadcopter follows the segway close enough to never lose sight of it. The strict safety surveillance requirement induces a conservative strategy.

{% include youtubePlayer2.html %}

To contrast this behaviour, the quadcopter is next given the surveillance task of $\LTLglobally \LTLfinally p_1$. Informally, the quadcopter has to infinitely often see the target exactly. Additionally the quadcopter has a task requirement of having to infinitely often return to a "charging station". 

Due to the more enclosed  environment, the quadcopter can search the areas until it finds the segway. This allows the satisfaction of additional task objectives.

### Unreal+Airsim simulations
 In this example, we demonstrate the behaviour of the agent under a safety surveillance specification in an urban environment created using Unreal Engine 4.

{% include youtubePlayer3.html %}

We note that the agent never allows the target to leave its line of sight. In general, open urban environments as is depicted here require stricter surveillance specifications as it is much harder to `find' a target once it has been lost.
