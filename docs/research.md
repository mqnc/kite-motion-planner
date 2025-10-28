
This is half devlog, half summary and half author's diary.

Note that it is mainly about path planning for robot arms, not vehicles, but some resources or considerations might be useful as well there.

# Considerations

Path planning usually involves the following steps:

- virtually exploring the environment by testing [random] states and connecting them to other tested states
- finding a path through valid explored states
- smoothing the path
- generating a feasible velocity profile along the path

There are alternatives:

https://wiki.ros.org/Papers/ICRA2011_Kalakrishnan
https://www.ri.cmu.edu/pub_files/2009/5/icra09-chomp.pdf

The tricky bit is to guarantee that no collisions are missed. If we check the validity of a path between two states by sampling several states along it, there is always the possibility that a thin object or edge hits another one and the samples miss it.

There are two main approaches to deal with this:

## Conservative Advancement (CA)

http://graphics.ewha.ac.kr/FAST/FAST.pdf

https://www.researchgate.net/publication/220946699_CCQ_Efficient_Local_Planning_Using_Connection_Collision_Query/link/0046352373ac71bb82000000/download

This basically means that we measure the distance between objects and then we guarantee that we only advance so far that nothing is going to collide. However, ideally we explore the environment in joint space and it might be a bit tricky to determine how far the furthest traveling point travels in Cartesian space depending on our joint angle steps. If I understand correctly, the papers only assume linear motion and simple rotation of the links but end effector movement is more complicated. Consider the following scenario:

```
       !
  /    |     \
 /     |      \
/  ->  |  ->   \
\      |       /
 \     |      /
  \    |     /
  1    2     3
```

If an arm moves from state (1) to state (3), sweeping each individual limb from start to finish will miss the extra extension bit (! in the sketch).

A potentially feasible way would be this:

```
stretched robot arm:
j1         j2       j3   j4
o----------o--------o----o----c
     l1        l2     l3   l4
```
In any possible circumstance, the end effector can at maximum travel

Δφ1 * (l1+l2+l3+l4) + Δφ2 * (l2+l3+l4) + Δφ3 * (l3+l4) + Δφ4 * l4

so we can just inflate the end-effector that much.

However, this likely overestimates the actual distances largely and the method becomes overly conservative and slow.

Another problem could be that this method will never actually detect a collision. Since it is guaranteed that any step we can take will not collide, we will just asymptotically approach a collision state. Thus we just have to stop trying to simulate a motion after a certain amount of steps or when a certain limit distance is reached. However, this might still work with some planner strategies.

## Continuous Collision Detection (CCD)

We create an object that represents the sweep of our object of interest from its start to its target and see if the sweep object collides with anything. For instance, if we want to move a sphere from A to B then the sweep object would be a capsule.

http://www.cs.unc.edu/techreports/03-038.pdf

In this paper they construct a mesh for the volume that is actually swept by a robot link. However, they also have to sample along the way and then they make an error estimate. The whole thing is also crazy complicated.

## The Thing

The thing is that any of these methods (even the simplest sampling) will work if only we put sufficient collision margin around the robot. However, it is difficult to estimate this margin in order to guarantee freedom from collision in all situations but at the same time not restrict the motion way too much.

# Libraries

## OMPL - Open Motion Planning Library

https://ompl.kavrakilab.org/

- library for sampling-based motion planning algorithms
- abstract exploration of state graphs, knows nothing about physics or collision
- provides a ton of planners: https://ompl.kavrakilab.org/planners.html
- used by ros
- looks well-documented
- provides simple defaults for each aspect but everything can be overwritten

OMPL requires Boost (version 1.58 or higher), CMake (version 3.5 or higher), and Eigen (version 3.3 or higher). Some additional features are available if ODE is installed. OMPL.app requires in addition Assimp, Open GL libraries and header files, libccd (version 2.0.0 or higher), and FCL (version 0.5.0 or higher). To be able to generate python bindings you need to install the Python library and header files and Py++. The OMPL.app GUI requires PyQt5 (including its OpenGL bindings) and PyOpenGL. Finally, you need a C++14 compiler (g++-5 or newer).

## FCL - The Flexible Collision Library

https://github.com/flexible-collision-library/fcl

https://gamma.cs.unc.edu/FCL/fcl_docs/webpage/pdfs/fcl_icra2012.pdf

- library wrapping several collision checking and avoiding algorithms into the same interface
- Collision detection: detecting whether the two models overlap, and optionally, all of the triangles that overlap
- Distance computation: computing the minimum distance between a pair of models, i.e., the distance between the closest pair of points
- Tolerance verification: determining whether two models are closer or farther than a tolerance distance
- Continuous collision detection: detecting whether the two moving models overlap during the movement, and optionally, the time of contact
- Contact information: for collision detection and continuous collision detection, the contact information (including contact normals and contact points) can be returned optionally
- implements the papers that were mentioned above (CA and CCD)
- CCD only sweeps linear translation and rotation though but the end effector moves in a more complicated manner
- someone on the internet stated that it is ten times slower than bullet

Before compiling FCL, please make sure Eigen and libccd (for collision checking between convex objects and is available here https://github.com/danfis/libccd) are installed. For libccd, make sure to compile from github version instead of the zip file from the webpage, because one bug fixing is not included in the zipped version.

## RL - Robotics Library

https://www.roboticslibrary.org/

https://github.com/roboticslibrary/rl

- looks well-maintained and well-documented
- I hardly find anything about it though

C++ library for robotics
The Robotics Library (RL) is a self-contained C++ library for robot kinematics, motion planning and control. It covers mathematics, kinematics and dynamics, hardware abstraction, motion planning, collision detection, and visualization.

It is being used by several research projects (e.g., JAHIR, JAMES, JAST, SMErobotics) and in education, available under a BSD license, and free for use in commercial applications.

RL can be run on all machines from real-time patched Linux to Windows desktop PCs. It uses CMake as a build system, may be compiled with GCC and Visual Studio.

## Bullet

https://pybullet.org/wordpress/

https://github.com/bulletphysics/bullet3

- most popular physics library
- can also perform just collision checks without dynamics
- convex hull with collision margin particularly useful
- no distance between objects though
- yes, there is: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=5538

## libccd

https://github.com/danfis/libccd

- library for computing the distance between convex shapes
- used by bullet, fcl, ode

# Short Term Concept

- use bullet for collision checks and (unsafe) sampling for collision checks along paths
- use ompl for path finding through the graph and simplification of the path
- perform p2p motion between the way points

# Long Term Concept

## Collision Check for Joint Steps

If we enclose every link with a capsule, we can propagate from the end effector to the base and construct a volume that encloses any possible joint motion from start to finish.

```
joint
o----------o ee start
 \  link   |
  \        |
   \      _o helper
    \  _.'
     o'
   ee target
```

Bullet can handle collision shapes represented by the convex hull around a couple of spheres. Let us imagine that our robot has just one joint and one link. If we have a capsule around the link and start and target values for the joint angle, we can construct such a collision shape for the complete motion as depicted. The helper point is necessary because if we connect start and target with a straight line, the curve of the actual motion will lie outside of that.

If we now mount this complete thing to another robot with one joint and one link, we can do the same thing again for every point in our new object. With this we propagate through all six joints of the robot and we can be absolutely certain that the robot will stay within this volume for this motion.

Ok ok, sketch. For simplicity, we imagine there is no end effector.

![concept.svg](concept.svg)

First we construct the bounding volume for the motion of the second link (dark green) as if nothing else moved. We end up with a volume defined by the convex hull around four spheres. Now we consider the first link. It sweeps its own capsule through space (blue) as well as the whole thing that we have constructed for the second link. We continue this consideration through the whole robot arm.

This causes two types of overestimation:

- In reality we drive a curve through joint space but we cover the whole cuboid from all start joint values to all target joint values.
- The helper point is further out than the actual motion (yellow corners).

Both aspects should be acceptable even for pretty large joint motion.

The drawback is that the number of points for such a volume grows exponentially with the number of joints that we consider. If we have n points for the end effector and one capsule for each link, the resulting volume will have

(((((( n \*3+1) \*3+1) \*3+1) \*3+1) \*3+1) \*3+1) = 729 n + 364 spheres

However, if we only move one or two joints at a time (at least during exploration of the environment for tree or road map construction), this is super efficient because every joint that does not move will not cause a multiplication with 3. And we can cover a large motion in just one step with this. Furthermore, we can maybe discard any sphere that does not contribute to the convex hull of its volume in each step along the robot. This might reduce the number of spheres dramatically but pruning the convex hull might also be expensive, needs testing.

## Road Map

If the changes to the environment are small during operation, we can just block the complete changing volume and construct a road map in the remaining free volume.

If we sample each joint in n steps, we end up with n^6 different robot configurations. These configurations are the nodes of our road map graph. Now we try to connect the nodes between which only one joint changes a step. So each node has 12 neighbors and we end up with n^6*12/2 connections. For one connection we need to perform one simulated test motion to see if it is collision-free.

```
n       step    tests   memory
8       45°     1.6 M   200 KB
12      30°     18 M    2.3 MB
16      22.5°   100 M   12.6 MB
24      15°     1.2 G   144 MB
32      11.25°  6.5 G   806 MB
36      10°     13.1 G  1.64 GB
48      7.5°    73.4 G  9.2 GB
```

Note that the UR5 has 720° range in each joint but we only need to sample one full circle because after one revolution of a joint the robot is in the exact same configuration as before. Joint limit hits will have to be regarded in addition to the collision graph but are trivial.

Once this road map his constructed (which will probably take some time and has to be redone every time the environment changes) we can very quickly, reliably and deterministically find the optimal path using the A* algorithm. Since this is a regular grid in joint space, it is also trivial to find the nearest point to the current robot position in the graph.

## Path Smoothing

The resulting path of the previous step will most likely be a zic zac. It can be smoothened iteratively by bringing the position of each way point closer to the average of its neighbors in each step and performing a complete collision check along the path in every iteration. This has been done in another project and it worked very well.

## Velocity Profile

Determining a velocity profile along a trajectory that does not exceed any velocity or acceleration limits is somewhat tricky. In a first step, we have to simulate the trajectory forwards and consider only positive acceleration limits, meaning we accelerate only slowly but we can stop in an instance. We store the maximum velocity for each step along the trajectory. In a second step, we do a reverse simulation. Now we consider the maximum deceleration (which is acceleration reverse in time) and take care that we never overshoot the previously determined maximum velocity. And we generate a new maximum velocity for each trajectory step. When we actually move the robot, we just have to obey the found velocity limits.
