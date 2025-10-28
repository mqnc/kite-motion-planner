# Scene Description

## Warning

As this document is not generated automatically, it is probably already out of date. Please refer to the implementation of the scene preprocessor for reference.

## Language

Considered:
* JSON: no comments, no inf, no NaN, quotes annoying ​
* YAML: no ‒ [YAML: probably not so great after all](https://www.arp242.net/yaml-config.html)
* TOML: no ‒ [Why not TOML?](https://dev.to/siddharthshyniben/why-not-toml-1fj9)
* XML: just no. ​
* Lua: Yes, good idea, very tempting but too much ​

Choice: [JSON5](https://json5.org/) ‒ like JSON but all drawbacks fixed!
```js
{
  // comments
  unquoted: 'and you can quote me on that',
  singleQuotes: 'I can use "double quotes" here',
  lineBreaks: "Look, Mom! \
No \\n's!",
  hexadecimal: 0xdecaf,
  leadingDecimalPoint: .8675309, andTrailing: 8675309.,
  positiveSign: +1,
  trailingComma: 'in objects', andIn: ['arrays',],
  "backwardsCompatible": "with JSON",
}
```

## Scene
```js
Scene = {
	version: [1, 2, 3],
	models: { // templates for robots and passive objects
		myRobotType: RobotModel,
		myStoneType: RigidBody
	},
	objects: { // instances of models
		myRobot: Object,
		myStone: Object
	},
	collisionIgnoreGroups: {
		robotAndStone: ["myRobot", "myStone"]
	},
	trajectories: {
		myTrajectory: Trajectory
	}
}
```

## Object
```js
Object = {
	model: "myRobotType",
	parent: "__WORLD__"
	pose: Transform
	jointValues: {
		elbow: 0.0,
		knuckle1: -1.57,
		knuckle2: 1.5
	}
}
```

## RobotModel
```js
RobotModel = {
	joints: {
		elbow: Joint,
		knuckle1: Joint,
		knuckle2: Joint
	}, 
	links: { // trafos in links reference joints
		upperArm: Link,
		lowerArm: Link,
		finger1: Link,
		finger2: Link
	},
	partModels: { // templates for links
		armModel: RigidBody,
		fingerModel: RigidBody
	},
	collisionIgnoreGroups: {
		arm: ["upperArm", "lowerArm"],
		hand: ["lowerArm", "finger1", "finger2"]
	},
	constraints: {
		// the dot product of the constraint coefficients
		// and the corresponding joint values is kept constant
		grab: { // mirror finger angle
			knuckle1: 1.0,
			knuckle2: -1.0
		}
	}
}
```

## Joint
```js
Joint = {
	minValue: -3.14,
	maxValue: 3.14,
	maxVelocity: 2.0,
	maxAcceleration: 3.0,
	maxDeceleration: 4.0,
	maxJerk: 5.0,
	maxEffort: 6.0
}
```

## Link
```js
Link = {
	parent: "myParent",
	pose: Transform,
	parts: ["myRigidBody1", "myRigidBody2"],
}
```

## RigidBody
```js
RigidBody = {
	mass: 1.5,
	centerOfMass: [1.0, 2.0, 3.0],
	inertia: {ixx: 4.0, iyy: 5.0, izz: 6.0, ixy: 0.0, ixz: 0.0, iyz: 0.0},
	shapes: {
		myBox: Shape,
		myCylinder: Shape,
		...
	}
}
```

## Shapes
```js
Shape = {
	geometry: Sphere / Box / Cylinder / OrangeNet / Mesh,
	collides: true,
	margin: 0.03,
	// when a contacting object starts sticking (for grippers):
	stickyForce: 2.0,
	visuals: {...} // additional info for visualizing the shape
}

Sphere = {
	center: [0.0, 0.0, 0.0],
	radius: 1.0
}

Box = {
	pose: Transform, // only static transform without references
	size: [1.0, 2.0, 3.0]
}

Cylinder = {
	center: [0.0, 0.0, 0.0],
	radius: 1.0,
	height: 1.0
	axis: [0.0, 0.0, 1.0]
}

OrangeNet = { // maybe IsoSphereHull?
	points: [
		[1.0, 2.0, 3.0],
		[7.0, 5.0, 3.0],
		[0.0, 0.0, 0.0]
	]
	radius: 1.0
}

Mesh = {
	vertices: [
		[1.0, 2.0, 3.0],
		[7.0, 5.0, 3.0],
		[2.0, 4.0, 1.0],
		[0.0, 0.0, 0.0]
	],
	faces: [
		[0, 1, 2],
		[1, 2, 3]
	]
}
```

## Transforms

Note the missing {} around subclasses, transform types are distinguished by their attributes only. For example:

```js
{xyz: [1, 2, 3], qxyzw: [0, 0, 0, 1]}
{dh: {a: 1, alpha:3.14, theta:0, s:0}}
```

```js
Transform = {PositionAndOrientation / Matrix4x4 / DenavitHartenberg}

PosAndOri =
	(TranslationVector / AxisDistance)
	and
	(Quaternion / Matrix3x3 / Euler / AxisAngle / RotationVector)

Matrix4x4 =
	rows4x4: [
		[1.0, 0.0, 0.0, 5.0],
		[0.0, 1.0, 0.0, 6.0],
		[0.0, 0.0, 1.0, 7.0],
		[0.0, 0.0, 0.0, 1.0]
	]
	or
	cols4x4: [
		[1.0, 0.0, 0.0, 0.0],
		[0.0, 1.0, 0.0, 0.0],
		[0.0, 0.0, 1.0, 0.0],
		[5.0, 6.0, 7.0, 1.0]
	]

DenavitHartenberg = 
	dh: {
		a: Value,
		alhpa: Value,
		theta: Value,
		s: Value
	}

TranslationVector =
	xyz: [Value, Value, Value]

AxisDistance =
	axisDistance: {
		axis: [1.0, 0.0, 0.0],
		distance: Value
	}

Quaternion =
	qxyzw: [0.0, 0.0, 0.0, 1.0]
	or
	qwxyz: [1.0, 0.0, 0.0, 0.0]

Matrix3x3 =
	rows3x3: [
		[1.0, 0.0, 0.0],
		[0.0, 1.0, 0.0],
		[0.0, 0.0, 1.0]
	]
	or
	cols3x3: [
		[1.0, 0.0, 0.0],
		[0.0, 1.0, 0.0],
		[0.0, 0.0, 1.0]
	]

Euler =
	euler: {
		// https://en.wikipedia.org/wiki/Euler_angles
		// extrinsic rotations, x then y then z
		x0y0z0: [Value, Value, Value]
		or
		// intrinsic rotations, z then y then x
		z0y1x2: [Value, Value, Value]
		// or any of the 24 possibilities
	}

AxisAngle =
	axisAngle: {
		axis: [1.0, 0.0, 0.0],
		angle: Value
	}

RotationVector =
	rotvec: [1.0, 2.0, 3.0]

Value = 
	1.337
	or
	{joint: "elbow"}
	or
	{
		trajectory: "myZigzag",
		index:2 // for multi-dimensional trajectories
	}
```

## Trajectory
```js
Trajectory = {
	controlPoints: [
		ControlPoint,
		ControlPoint,
		...		
	],
	dtDefault: 0.1
	repeat: true / {t: 10.0} / {dt: 0.1}
}

ControlPoint = {
	t: 1.0 / dt: 0.1 // optional; t is absolute, dt is relative to last cp
	f: 5.0 / f: [3.0, 4.0, 5.0]
	df: -0.5 / df: [...] // optional, makes trajectory cubic spline
	ddf: 0.0 / ddf: [...] // optional, makes trajectory quintic spline
}
```

## References to other Files

... can be used instead of any value. See [RFC3986](https://datatracker.ietf.org/doc/html/rfc3986) and [RFC6901](https://datatracker.ietf.org/doc/html/rfc6901).
```js
{uri: "file://./myRobotDefs.json#/WallE"}
```

## Notes

### Why?
This format was developed with scene description for virtual scene management, trajectory planning and simulation in mind. URDF/SDF was not enough.

### Flexibility VS Simplicity
One major goal was to support all common kinds of transformation notation, so that nobody has to convert anything by hand anymore. However, this flexibility entails that every tool that is supposed to read a scene also needs to implement every single format.

### Preprocessing
The proposed solution to the aforementioned problem is a scene preprocessor. This serves multiple purposes:
* Input validation: do all referenced objects exist, are there no negative masses, etc.
* Sanitization: Normalize all quaternions and axes
* Include external references
* Convert into some canonical subset of the scene language

Any program that needs to load a scene can feed it to the preprocessor. It can then be sure that the output is valid and it only needs to implement a subset of all the scene description flexibility, for instance only position vector and quaternions for static poses.