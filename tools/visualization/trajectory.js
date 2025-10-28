import { checkFields, Referenceable } from "./utils.js"

class Trajectory {
	constructor() {
		this.controlPoints = []
		this.repeat = NaN
		this.needsUpdate = false
		this.continuity = undefined
		this.intervals = []
		this.coefficients = []
	}
	repeatAt(t) {
		this.repeat = t
	}
	addControlPoint(t, f, df = NaN, ddf = NaN) {
		if (this.continuity === undefined) {
			if (isNaN(df) && isNaN(ddf)) { this.continuity = 0 }
			else if (!isNaN(df) && isNaN(ddf)) { this.continuity = 1 }
			else if (!isNaN(df) && !isNaN(ddf)) { this.continuity = 2 }
			else { throw new Error("invalid control point") }
		}
		else {
			if (this.continuity == 0) { console.assert(isNaN(df) && isNaN(ddf)) }
			else if (this.continuity == 1) { console.assert(!isNaN(df) && isNaN(ddf)) }
			else if (this.continuity == 2) { console.assert(!isNaN(df) && !isNaN(ddf)) }
		}
		this.controlPoints.push({ t, f, df, ddf })
		if (
			this.controlPoints.length >= 2
			&& this.controlPoints.at(-1).t < this.controlPoints.at(-2).t
		) {
			this.controlPoints.sort((a, b) => b.t - a.t)
		}
		this.needsUpdate = true
	}
	tStart() { return this.controlPoints[0].t }
	tEnd() {
		return isNaN(this.repeat) ?
			this.controlPoints.at(-1).t
			: this.repeat
	}
	duration() { return this.tEnd() - this.tStart() }
	computeCoefficients() {
		this.intervals = []
		this.coefficients = []

		if (!isNaN(this.repeat)) {
			// append first control point temporarily
			this.controlPoints.push({
				t: this.repeat,
				f: this.controlPoints[0].f,
				df: this.controlPoints[0].df,
				ddf: this.controlPoints[0].ddf
			})
		}

		if (this.continuity == 0) {
			for (let k = 0; k + 1 < this.controlPoints.length; k++) {
				const T = this.controlPoints[k + 1].t - this.controlPoints[k].t
				const f0 = this.controlPoints[k].f
				const f1 = this.controlPoints[k + 1].f
				this.intervals.push(T)
				this.coefficients.push([f0, f1 - f0])
			}
		}
		else if (this.continuity == 1) {
			for (let k = 0; k + 1 < this.controlPoints.length; k++) {
				const T = this.controlPoints[k + 1].t - this.controlPoints[k].t
				const f0 = this.controlPoints[k].f
				const df0_T = this.controlPoints[k].df * T
				const f1 = this.controlPoints[k + 1].f
				const df1_T = this.controlPoints[k + 1].df * T

				this.intervals.push(T)

				// q = 0..1 // progress within the current interval
				// f = c3*q^3 + c2*q^2 + c1*q + c0
				// [f(0), df(0), f(1), df(1)] = M * [c3, c2, c1, c0]
				// determine M using maths, invert it and you get the coefficients below
				// the scaling with T is to account for the actual interval length
				this.coefficients.push([
					f0,
					df0_T,
					-3 * f0 - 2 * df0_T + 3 * f1 - df1_T,
					2 * f0 + df0_T - 2 * f1 + df1_T,
				])
			}
		}
		else if (this.continuity == 2) {
			for (let k = 0; k + 1 < this.controlPoints.length; k++) {
				const T = this.controlPoints[k + 1].t - this.controlPoints[k].t
				const T2 = T * T
				const f0 = this.controlPoints[k].f
				const df0_T = this.controlPoints[k].df * T
				const ddf0_T2 = this.controlPoints[k].ddf * T2
				const f1 = this.controlPoints[k + 1].f
				const df1_T = this.controlPoints[k + 1].df * T
				const ddf1_T2 = this.controlPoints[k + 1].ddf * T2

				this.intervals.push(T)

				// q = 0..1 // progress within the current interval
				// f = c5*q^5 + c4*q^4 + c3*q^3 + c2*q^2 + c1*q + c0
				// [f(0), df(0), ddf(0), f(1), df(1), ddf(1)] = M * [c5, c4, c3, c2, c1, c0]
				// determine M using maths, invert it and you get the coefficients below
				// the scaling with T is to account for the actual interval length
				this.coefficients.push([
					f0,
					df0_T,
					0.5 * ddf0_T2,
					-10 * f0 - 6 * df0_T - 1.5 * ddf0_T2 + 10 * f1 - 4 * df1_T + 0.5 * ddf1_T2,
					15 * f0 + 8 * df0_T + 1.5 * ddf0_T2 - 15 * f1 + 7 * df1_T - ddf1_T2,
					-6 * f0 - 3 * df0_T - 0.5 * ddf0_T2 + 6 * f1 - 3 * df1_T + 0.5 * ddf1_T2
				])
			}
		}

		if (!isNaN(this.repeat)) {
			// remove temporarily appended control point
			this.controlPoints.pop()
		}

		this.needsUpdate = false
	}
	eval(t) {
		if (this.needsUpdate) { this.computeCoefficients() }
		if (isNaN(this.repeat)) {
			if (t < this.tStart()) { t = this.tStart() }
			if (t > this.tEnd()) { t = this.tEnd() }
		}
		else {
			const t0 = this.tStart()
			const d = this.duration()
			// the extra + d % d is to handle negative values properly
			t = (((t - t0) % d) + d) % d + t0
		}
		let k = 0
		while (k + 1 < this.controlPoints.length && this.controlPoints[k + 1].t < t) { k++ }
		let q = (t - this.controlPoints[k].t) / this.intervals[k]
		let c = this.coefficients[k]
		if (this.continuity == 0) {
			return c[1] * q + c[0]
		}
		else if (this.continuity == 1) {
			return c[3] * Math.pow(q, 3) + c[2] * Math.pow(q, 2) + c[1] * q + c[0]
		}
		else if (this.continuity == 2) {
			return c[5] * Math.pow(q, 5) + c[4] * Math.pow(q, 4) + c[3] * Math.pow(q, 3)
				+ c[2] * Math.pow(q, 2) + c[1] * q + c[0]
		}
	}
}

export function jsonToTrajectoryReferenceables(json) {
	checkFields(json, ["controlPoints"], ["dtDefault", "repeat"])

	let referenceables = []

	const f0 = json.controlPoints[0].f
	let isArray = Array.isArray(f0)
	let dimensions = isArray ? f0.length : 1
	let repeatAt

	for (let dim = 0; dim < dimensions; dim++) {
		let trajectory = new Trajectory()
		let dtDefault = json.dtDefault ?? 1
		let t = 0
		for (let cp of json.controlPoints) {
			if ("t" in cp) { t = cp.t }
			else {
				if (cp !== json.controlPoints[0]) {
					if ("dt" in cp) { t += cp.dt }
					else { t += dtDefault }
				}
			}
			if (isArray) {
				trajectory.addControlPoint(
					t,
					cp.f[dim],
					cp.df ? cp.df[dim] : NaN,
					cp.ddf ? cp.ddf[dim] : NaN
				)
			}
			else {
				trajectory.addControlPoint(
					t,
					cp.f,
					cp.df ?? NaN,
					cp.ddf ?? NaN
				)
			}
		}

		if ("repeat" in json && repeatAt === undefined) {
			if (json.repeat === true) {
				repeatAt = trajectory.tEnd() + dtDefault
			}
			else if (json.repeat === false) {
				repeatAt = NaN
			}
			else if ("t" in json.repeat) {
				repeatAt = json.repeat.t
			}
			else if ("dt" in json.repeat) {
				repeatAt = trajectory.tEnd() + json.repeat.dt
			}
			else {
				throw new Error("invalid repetition specification")
			}
		}

		trajectory.repeatAt(repeatAt)

		referenceables.push(
			new Referenceable(value => trajectory.eval(value))
		)
	}

	return isArray ? referenceables : referenceables[0]
}

