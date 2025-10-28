
# this is a failed experiment for building a path tracker with limited velocity, acceleration and jerk



import numpy as np
import matplotlib.pyplot as plt

dqForce = 1e-5
vMax = 1
aMax = 5
jMax = 10

def display(trajectory, dt):
	time = np.arange(0, len(trajectory), 1) * dt
	trajectory = np.array(trajectory)

	# Calculate derivatives
	first_derivative = np.diff(trajectory) / dt
	second_derivative = np.diff(first_derivative) / dt
	third_derivative = np.diff(second_derivative) / dt

	# Plotting the trajectory and its derivatives
	plt.figure(figsize = (10, 8))

	plt.subplot(411)
	plt.plot(time, trajectory, label = 'Trajectory', marker = ".")
	plt.ylim(0, 1)
	plt.ylabel('Position')
	plt.legend()

	plt.subplot(412)
	plt.plot(time[1:], first_derivative, label = 'Speed', marker = ".")
	plt.ylim(-1.1 * vMax, 1.1 * vMax)
	plt.ylabel('Speed')
	plt.legend()

	plt.subplot(413)
	plt.plot(time[2:], second_derivative, label = 'Acceleration', marker = ".")
	plt.ylim(-1.1 * aMax, 1.1 * aMax)
	plt.ylabel('Acceleration')
	plt.legend()

	plt.subplot(414)
	plt.plot(time[3:], third_derivative, label = 'Jerk', marker = ".")
	plt.ylim(-1.1 * jMax, 1.1 * jMax)
	plt.xlabel('Time')
	plt.ylabel('Jerk')
	plt.legend()

	plt.tight_layout()
	plt.show()

def curve(q):
	return q

qStart = 0
qTarget = 1

dt = 0.001
nIts = 40

def dqMax_for_q(qArray, dqMaxArray, q):
	return np.interp(q, qArray, dqMaxArray)

################################################################################

q = qStart
p = curve(q)
v = 0

qArray = [q]
dqMaxArray = [dqForce]

while q < qTarget:

	qLeast = min(q + dqForce, qTarget)
	qMost = qTarget

	for it in range(nIts):
		qTest = qMost if it == 0 else (qLeast+qMost) / 2
		pTest = curve(qTest)
		vTest = (pTest-p) / dt
		aTest = (vTest-v) / dt

		if ( #
			abs(vTest) <= vMax # velocity in bounds
			and (abs(aTest) <= aMax or vTest*aTest < 0) # acceleration in bounds or velocity changes towards 0
		):
			qLeast = qTest
			if (it == 0):
				break
		else:
			qMost = qTest

	q_ = q
	p_ = p
	v_ = v

	q = qLeast
	qArray.append(q)
	dqMaxArray.append(max(q - q_, dqForce))

	p = curve(q)
	v = (p-p_) / dt

display([curve(q) for q in qArray], dt)

################################################################################

qArrayPrev = qArray
dqMaxArrayPrev = dqMaxArray

q = qTarget
p = curve(q)
v = 0

qArray = [q]
dqMaxArray = [dqForce]

while q > qStart:

	qLeast = max(q - dqForce, qStart)
	qMost = max(qStart, q - dqMax_for_q(qArrayPrev, dqMaxArrayPrev, q))

	for it in range(nIts):
		qTest = qMost if it == 0 else (qLeast+qMost) / 2
		pTest = curve(qTest)
		vTest = (pTest-p) / dt
		aTest = (vTest-v) / dt

		if ( #
			abs(vTest) <= vMax # velocity in bounds
			and (abs(aTest) <= aMax or vTest*aTest < 0) # acceleration in bounds or velocity changes towards 0
		):
			qLeast = qTest
			if (it == 0):
				break
		else:
			qMost = qTest

	q_ = q
	p_ = p
	v_ = v

	q = qLeast
	qArray.append(q)
	dqMaxArray.append(max(abs(q - q_), dqForce))

	p = curve(q)
	v = (p-p_) / dt

qArray.reverse()
dqMaxArray.reverse()

display([curve(q) for q in qArray], dt)

plt.plot(qArray, np.gradient(np.gradient([curve(q) for q in qArray])/dt)/dt, marker = ".")
plt.show()

################################################################################
# following section doesnt work
################################################################################

qArrayPrev = qArray
dqMaxArrayPrev = dqMaxArray

q = qStart
p = curve(q)
v = 0
a = 0

qArray = [q]
dqMaxArray = [dqForce]

while q < qTarget:

	qLeast = min(q + dqForce, qTarget)
	qMost = min(qTarget, q + dqMax_for_q(qArrayPrev, dqMaxArrayPrev, q))

	for it in range(nIts):
		qTest = qMost if it == 0 else (qLeast+qMost) / 2
		pTest = curve(qTest)
		vTest = (pTest-p) / dt
		aTest = (vTest-v) / dt
		jTest = (aTest-a) / dt

		if ( #
			abs(aTest) <= aMax # acceleration in bounds
			and (abs(jTest) <= jMax or aTest*jTest < 0) # jerk in bounds or acceleration changes towards 0
		):
			qLeast = qTest
			if (it == 0):
				break
		else:
			qMost = qTest

	q_ = q
	p_ = p
	v_ = v
	a_ = a

	q = qLeast
	qArray.append(q)
	dqMaxArray.append(q - q_)

	p = curve(q)
	v = (p-p_) / dt
	a = (v-v_) / dt

display([curve(q) for q in qArray], dt)
