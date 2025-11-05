
## General Notes

There were the mysteries:
why is there this regular zig zag in the motion? -> results from only moving some joints at a time when the planner tries to connect the trees

why does the start tree look different from the goal tree?
	bool validMotion = treeGrowingInfo.isStartTree ? si_->checkMotion(nearestMotion->state, dstate) :
		si_->isValid(dstate) && si_->checkMotion(dstate, nearestMotion->state);
	since sweeping checks the complete motion, this is actually not necessary...

## Todo

if we sample trajectories by the desired dt, collision checks and ik can take forever if the trajectory is slow. we need to sample by a test-dl and then interpolate in the end

Find cycles in parent child relationship.

I wanted to change an orangenet to a cylinder, changed "points" to "center", noticed I'm in the wrong file and left it. When I worked with the accidentally changed file, the planner did not find points, created an empty convex hull and bullet caused a segfault. A lot of input checks will have to be done...

the c++ scene loader needs to handle the new trajectory repeat format

why is there no margin in the pillars in the robot with pillars debug scene

"robot1": {
	"model": "ur5e",
	"parent": "WORLD",
	"pose": { "xyz": [ 0, 0, 0 ], "qxyzw": [ 0, 0, 0, 1 ] },
	"jointValues": {
		"base": 3.14,
		"shoulder": -0.6,
		"elbow": { "trajectory": "wave" },
		"wrist1": 0.4,
		"wrist2": 0.5,
		"wrist3": 0.6
	}
}
the trajectory link expects a number

the new collision ignore group management has to be reflected in js

try without hull optimization

why is debug as fast as release
-> this came from the bullet initialization of collisionWorld, it is now fixed by making some things static members

need an alternative to DH for specifying a free translation together with a free rotation about an axis I think

## Nice-to-haves

Inverse kinematics: IK is currently done in the python wrapper for UR robots only. The challenge is that different robot architectures need different algorithms and also different ways to handle the solutions. A standard industrial robot has 2, 4 or 8 solutions (depending on whether it can invert its elbow and do overhead operations). The URs have 512 (since each joint can do 2 full turns) which often need filtering because it is too expensive to consider them all in further operations. The Panda has 7 joints and thus infinitely many solutions. We either need a plugin system for IK modules for different robots (.so file? lua? Tiny C Compiler?) or we need the most common ones hard-coded in the planner and use numeric IK otherwise. Then, based on that, we can do linear interpolation with speed checks, which is currently also done in the python wrapper.

Multi threading: This can potentially accelerate the planning process by a factor proportional to the number of threads utilized. However, depending on how deep we want to embed this, this might be a bigger refactory since it is currently not considered. Easiest would be to start 16 planners with different random seeds and see which one finishes first, that would also be the least efficient since the exploration trees do not benefit from each other. Most difficult and best would be if a single exploration tree can grow 16 branches at once. Even more challenge can be introduced by still wanting it to be deterministic.

Workspace Visualization: (see https://publikationen.bibliothek.kit.edu/1000065120) This is useful for easier robot cell design, robot placement and live robot control. This can also generate nice visuals for promotion purposes. We can make this more complicated if we want to factor collisions in, the linked version only considers joint limits and robot length.

Importer for URDF: The collision primitives for hierarchical sweeping must be convex hulls around equally-sized spheres which is not supported by URDF. So we either need to model each new robot by ourselves or create some URDF conversion helper.

RRT considering joint space cuboids: If we check a motion from joints(0°, 0°, 0°, 0°, 0°, 0°) to joints(30°, 30°, 30°, 30°, 30°, 30°) and it is collision-free, we can also be sure that joints(30°, 0°, 0°, 0°, 0°, 0°), joints(0°, 30°, 0°, 0°, 0°, 0°) and all other combinations are collision free, the whole cuboid in joint space gets checked. So we can add each of the corners to the tree growth in the RRT algorithm, which might speed it up a lot.

Optimize convex hull generation: Each link consists of inflated convex hulls around support points (ideally just 2 for a sausage). When we sweep a joint, the amount of support points that hang on that joint triples. Moving another joint further up triples them again. Say we have a 6DoF robot where each link has two support points and we move all joints at once. That means we get `((((2*3)+2)*3)+2)*3… = 2186` support points in the end. Since we are only interested in convex hulls, we can reduce that number by discarding points that do not contribute to the convex hull along the way. However, computing a convex hull is also expensive and might not be worth it. But that algorithm also does not need to be perfect, maybe we can find a cheap way to only discard the majority of points that do not contribute to the convex hull.

## Path Tracing with Limited Velocity, Acceleration and Jerk

When following a path in space xyz(s) or joint values q1..7(s), it would be desirable to find a time optimal parameterization s(t) so that up to three derivatives of the resulting trajectory can be limited. Parametrizing a path with limits on s'(t) and s"(t) is doable. We can simulate the path forwards, accelerate as much as allowed while allowing instant braking and afterwards simulate the path backwards, accelerating (backwards in time so actually braking) as hard as allowed while staying below the previously found trajectory. However, when a jerk limit comes into play, this does not work anymore, an insight resulting from two days of doodling and playing around in python. Even if it worked, the mapping from joint limitations to path limitations is intertwingled. So s"'max(t) depends on j'(s), j"(s) and j"'(s).

Right now, we are simulating the complete trajectory with a certain s(t) profile, check all limits and then scale the complete profile s(c*t) where c is constant. This removes the intertwingledness but lets us apply speed limitations from some critical spots on the whole trajectory which is not cool.

Now reading this paper:

https://www.researchgate.net/publication/334617431_Time-Optimal_Path_Tracking_for_Jerk_Controlled_Robots

Now putting the topic back on the shelf.

Now reaching for the shelf:

The thing is, continuous trajectories are right now stored with controlpoints containing f, df, ddf, with a quintic interpolation in between. A trajectory that is generated by ruckig will however spit out a different motion (although it is probably also a piecewise quintic polynomial, maybe we can split that into our format)

## btCompoundShape

btCompoundShape has a m_collisionMargin member, investigate what it does! if it overrides the collision margin of its children, that would seriously mess things up
Update: Looks like it is fine. It is not that it does nothing, if the margin is set to negative values or NaN, it will not work. But >0 it seems that every sub shape has its correct margin

maybe node.h needs a refactory, all the methods should maybe be free functions. on the other hand they also deeply work with the node members...


## Why the sweeping collision check is hierarchical
 
If we check the collisions of a motion by sampling, we can just construct the complete scene and test for collisions in every time step. However, if we sweep objects from one time step to another, we can create false positives. Imagine a hand with two fingers, which performs a motion in a way so that finger two ends up where finger one started, without the fingers moving with respect to the hand. The constructed sweeping volumes will clearly intersect, although the fingers never actually collide. To prevent this, we need to do a hierarchical collision check. Starting from the leaf nodes of the scene graph (fingers), we work our way towards the root (world node). First, we consider finger one, it does not collide, it is just a lonesome finger. Same for finger two. Then we investigate the hand, constructing the sweeping volumes for all attached moving parts. We said the fingers do not move with respect to the hand, so no sweeping, just one static collision check between the fingers. Now we can be sure that the fingers do not collide with each other, no matter what the hand does. So as we work our way up the hierarchy, we ignore any collisions inside the individual subtrees that we already checked. If we now move the hand in a way so that the sweeping volumes for the fingers intersect, it is no longer a problem, as it can be ignored.
