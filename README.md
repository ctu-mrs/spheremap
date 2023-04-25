# Spheremap server

## Dependencies

* [octomap](https://octomap.github.io/)
* [pcl_ros](http://wiki.ros.org/pcl_ros)

## SphereMap overview
SphereMap is a multi-layer dynamic map built online on-board a UAV. It allows weighing path length and distance from obstacles along the path for safety-aware planning, while also finding paths faster than a conventional occupancy grid (~1 order of magnitude or ~3 orders if precomputed paths are used).
The working principle is that at ~2Hz, the server server updates the map near the UAV's current position - filling free space with spheres, connecting them into a graph of intersecting spheres, and then segmenting this graph into approx. convex regions and computing and storing paths inside these regions for quick long-distance planning.
More information can be found in the publication  [SphereMap: Dynamic Multi-Layer Graph Structure for Rapid Safety-Aware UAV Planning](https://arxiv.org/pdf/2302.01833.pdf).

## Usage guide - running the server
The SphereMap server node requires two inputs (specified in the launch file): 
-occupancy mapper that produces an [OctoMap](https://octomap.github.io/) binary octomap_msgs::Octomap to the SphereMap server node 
-the current odometry as nav_msgs::Odometry.
If you connect these in the launch file, the SphereMap server node will initialize and start updating the SphereMap. There are various configuration params with which you can play with, described in the config file, and also multiple visualization topics for Rviz.

## Usage guide - safety-aware planning
Use the service GetSphereMapPath.srv for pathfinding in the SphereMap. It will transform the given start and goal points from the given frame to the SphereMap frame, try to find a path, and return it as a set of #TODO if successful. If you set the flag ignore_precomputed_paths to true, the planning will do A* over the entire SphereMap graph, which can give a better path than using the precomputed paths, but will be slower. If a start/goal point is not covered by the SphereMap, the planning will take the nearest sphere up to a distance specified in the config file, as the start node, so be aware of that.

The path cost is computed as in [publication](https://arxiv.org/pdf/2302.01833.pdf), and you can set the riskiness weight (epsilon in the publication, 'spheremap_planning_safety_weight' in the config file) as required. There is also a service for changing the weight mid-flight "SetSafetyPlanningParams.srv".

## Using the LTV-Map lightweight topological-volumetric map
Currently, the LTV-Map described in the publication is built along with the SphereMap and is published as a MarkerArray purely for visualization. Using the LTV-Maps for cooperative exploration planning was implemented in the SubT CHallenge and is described in this [other publication](https://arxiv.org/abs/2206.08185), but it was not implemented in the SphereMap server.

## Possible improvements
-adding service for planning to multiple goals at one service call (for dedicing between many goals)
-implementing LTV-Map sharing between UAVs and using them for cooperative navigation / exploration.
