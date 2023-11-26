# Lab 5: SLAM and Pure Pursuit

## I. Learning Goals

- SLAM
- Localization with Particle Filter
- Pure Pursuit Algorithm

## II. Running slam_toolbox on the car

Follow the instructions in class to run `slam_toolbox` to make a map of AI makerspace corridor leading towards the gym lockers so that you have a sufficiently long passage to demonstrate your run on it. Make sure you do not have a lot of crowd when you are mapping as they would interfere with your maps. If this is too much of a hassle, eve n n mapping AI makerspace is fine. Save the map as `ai_makerspace.pgm` and `ai_makerpace.yaml`.

## III. Localization with Particle Filter

Follow the instructions in class to run `particle_filter` on the car using the new map you've made inside or near AI makerspace.

## IV. Pure Pursuit Implementation

We have provided a skeleton for the pure pursuit node. As per usual, test your algorithm first in the simulator before you test it on the car. When you're testing in the simulator, use the groud truth pose provided by the sim as the localization. When you move to the car, use particle filter to provide localization.

As shown in the lecture, the curvature of the arc to track
can be calculated as:

<!-- ![](https://latex.codecogs.com/svg.latex?\gamma=\frac{2|y|}{L^2}) -->
$$\gamma=\frac{2|y|}{L^2}$$

## V. Logging Waypoints

There are several methods you can use to create waypoints for a specific map.

1. Recording a trajectory of joystick driven path. You can write a node that subscribe to the pose provided by the particle filter localization, and save the waypoints to a csv file. A similar script is provided [here](https://github.com/f1tenth/f1tenth_labs/blob/main/waypoint_logger/scripts/waypoint_logger.py). Note that this script is in ROS 1 and you'll have to write a ROS 2 node.

2. Find key points in the map (e.g. in the AI makerspace corridor, the turning points at th gym lockers) and create a interpolated spline that goes through all the corners. You can use functions such as `scipy.interpolate.splprep` and `scipy.interpolate.splev`. You can find more documentaion on these [here](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splprep.html) and [here](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splev.html#scipy.interpolate.splev).

Usually, you'll just save the waypoints as `.csv` files with columns such as `[x, y, theta, velocity, arc_length, curvature]`. With pure pursuit, the bare minimum is `[x, y]` positions of the waypoints. Another trick is that you can also smooth the waypoints if you decided to record it with the car. You can subsample the points you gathered and re-interpolate them with the `scipy` functions mentioned above to find better waypoints.

## VI. Visualizing Waypoints

To visualize the list of waypoints you have, and to visualize the current waypoint you're picking, you'll need to use the `visualization_msgs` messages and RViz. You can find some information [here](http://wiki.ros.org/rviz/DisplayTypes/Marker).

## VII. Deliverables

- **Deliverable 1**: Submit the map files ai_makerspace.pgm and ai_makerspace.yaml) that you've made using `slam_toolbox`.
- **Deliverable 2**: Commit your pure pursuit package to GitHub. Your commited code should run smoothly in simulation.
- **Deliverable 3**: Submit a link to a video on YouTube showing the real car following waypoints in or near AI makerspace. Show a screen recording of rviz. 

## VIII: Grading Rubric
- Compilation: **10** Points
- Running slam_toolbox and producing a map: **30** Points
- Running particle_filter: **20** Points
- Implementing pure pursuit: **30** Points
- Video: **10** Points
