# Kite Motion Planner
Proof-of-Concept Motion Planner for Robot Arms, written in C++

## About

This project is an experimental successor to the [Gestalt Motion Planner](https://github.com/mqnc/gestalt-motion-planner). It is centered around a concept that allows for very efficient motion validation that can theoretically guarantee collision-free motion (in contrast to the usual sampling or linear continuous collision detection approaches).

## Project Status

Despite having been developed to be used in production later on, this planner is still in an experimental state. The author is no longer working at Gestalt Automation GmbH where the project was initially started and is now maintaining it as a hobby.

## License

This project is released under the [PolyForm Noncommercial License 1.0.0](https://polyformproject.org/licenses/noncommercial/1.0.0/). For a commercial license contact [Gestalt Automation GmbH](https://www.gestalt-automation.com/en/contact).

## Key Concepts

Most of the basic functionality of this planner is taken from [the previous version](https://github.com/mqnc/gestalt-motion-planner). In essence, [bullet3](https://github.com/bulletphysics/bullet3) is used for collision detection, [OMPL](https://ompl.kavrakilab.org) is used for path finding and [ruckig](https://github.com/pantor/ruckig) is used for path smoothing to get jerk-limited trajectories.

This section discusses the two main ideas that are tested in this project.

### Hierarchical Sweeping

The core concept is somewhat similar to what is outlined in [this paper](https://www.cs.unc.edu/techreports/03-038.pdf) but utilizes some tweaks that make it slightly more conservative but much more efficient.

We approximate each moving part with convex hulls around one or more equally-sized spheres.

<!-- ![approximation](docs/approximation.png) -->
<p align="center"><img src="docs/approximation.png" alt="approximation" style="max-width: 100%; width: 400px;"></p>

In order to check a motion for collision, we start at the leaves of our kinematic tree. Let us consider a single finger which is modeled by a capsule shape: a convex hull around two spheres located at point $A$ and point $B$. We now want to move this finger using a joint from state 1 to state 2. For this we construct the volume that the finger will sweep through during the motion. We can then check this sweeping volume for collisions with the environment.

<p align="center"><img src="docs/construction.png" alt="construction" style="max-width: 100%; width: 600px;"></p>

If the joint is prismatic, we simply have to move points $A$ and $B$ along the translation axis and then construct the convex hull around $A_1$, $A_2$, $B_1$ and $B_2$. For revolute joints, we rotate each point about the joint axis from state 1 to state 2 but in addition to that we need a helper point ($A_h$ and $B_h$ in the sketch). We find it where the motion tangent at the start location of a point and the motion tangent at the target location of that point intersect. We then also include spheres around all helper points in the convex hull that covers our sweeping volume. This ensures that the resulting shape is guaranteed to completely enclose the finger at all times during motion.

Contrast this with traditional sampling and continuous collision detection approaches which can both result in missed collisions in the red areas:

<p align="center"><img src="docs/comparison.png" alt="comparison" style="max-width: 100%; width: 800px;"></p>

Now we have to propagate our way up the robot joint by joint to the robot base, always sweeping every volume we have constructed further down. As an example, here is a 6DoF robot where wrist, elbow and base each move by 30° and the step-by-step sweeping volume construction from end effector to base:

<p align="center"><img src="docs/sweep.png" alt="sweep" style="max-width: 100%; width: 800px;"></p>

This covers the main idea. For further discussion, check out [discussion](docs/sweep_discussion.md).

### Joint Projection Constraints

Some tasks require that the end effector keeps a certain orientation, for instance when holding a liquid. There are two popular approaches to deal with this:

1) Sampling lots of poses and keeping only those within some tolerance around the target orientation — this is computationally very expensive and also very unsexy.
2) Sampling in Cartesian space and computing the IK for every sample — also expensive and configuration flips have to be detected and avoided.

In this planner, we keep the end effector upright by working in a lower-dimensional slice of the joint space. Consider the following configuration:

<p align="center"><img src="docs/constraints.png" alt="constraints" style="max-width: 100%; width: 400px;"></p>

If we keep both `q5` as well as the sum `q2 + q3 + q4` constant, the flask can only stay perfectly level. This can be specified via the following list of linear constraint vectors: `[[0, 0, 0, 0, 1, 0], [0, 1, 1, 1, 0, 0]]`, meaning

```
0*q1 + 0*q2 + 0*q3 + 0*q4 + 1*q5 + 0*q6 = const.
0*q1 + 1*q2 + 1*q3 + 1*q4 + 0*q5 + 0*q6 = const.
```

Additionally, we can also define `q1 - q6` to remain constant (`[1, 0, 0, 0, 0, 0, -1]`), which would also fix the rotation of the flask about the vertical axis and thus completely fix its orientation in space during motion.

Each constraint vector restricts the robot motion to stay within a certain hyper plane (orthogonal to the vector) in joint space. The constants on the right side are determined from the current robot pose.

The advantages of this approach are:

* It works in joint space so it can even drive through singularities without issues.
* It even accelerates planning since it reduces the dimensionality of the search space.

The only disadvantage is that it only works for certain orientations, certain robot kinematics and when the robot is mounted horizontally (including upside down). Although that might seem very restrictive, it should cover the majority of use cases.

## Results

(to be done)
