import * as THREE from 'three'
import { checkFields } from "./utils.js"

export function jsonToDynamicMatrix(json, robotId, referenceables) {
	checkFields(json, [], ["xyz", "axisDistance",
		"qwxyz", "qxyzw", "cols3x3", "rows3x3", "euler", "axisAngle", "rotvec",
		"cols4x4", "rows4x4", "dh"])

	let trafo = new THREE.Matrix4()

	if ("dh" in json) {
		let rot_z_1 = new THREE.Matrix4()
		let trans_z_1 = new THREE.Vector3()
		let rot_x = new THREE.Matrix4()
		let trans_x = new THREE.Vector3()
		function compose() {
			trafo.multiplyMatrices(
				rot_z_1.clone().setPosition(trans_z_1),
				rot_x.clone().setPosition(trans_x),
			)
		}
		if (typeof (json.dh.theta) == "number") {
			rot_z_1.makeRotationZ(json.dh.theta)
		}
		else {
			jsonToRegisteredReference(json.dh.theta, robotId, referenceables,
				(value) => { rot_z_1.makeRotationZ(value); compose() }
			)
		}
		if (typeof (json.dh.s) == "number") {
			trans_z_1.setZ(json.dh.s)
		}
		else {
			jsonToRegisteredReference(json.dh.s, robotId, referenceables,
				(value) => { trans_z_1.setZ(value); compose() }
			)
		}
		if (typeof (json.dh.alpha) == "number") {
			rot_x.makeRotationX(json.dh.alpha)
		}
		else {
			jsonToRegisteredReference(json.dh.alpha, robotId, referenceables,
				(value) => { rot_x.makeRotationX(value); compose() }
			)
		}
		if (typeof (json.dh.a) == "number") {
			trans_x.setX(json.dh.a)
		}
		else {
			jsonToRegisteredReference(json.dh.a, robotId, referenceables,
				(value) => { trans_x.setX(value); compose() }
			)
		}
		compose()
	}
	else if ("cols4x4" in json) { throw new Error("cols4x4 not yet implemented") }
	else if ("rows4x4" in json) { throw new Error("rows4x4 not yet implemented") }

	else { // separate components for translation and rotation
		// translation
		if ("xyz" in json) {
			for (let i = 0; i < 3; i++) {
				if (typeof (json.xyz[i]) == "number") {
					trafo.elements[12 + i] = json.xyz[i]
				}
				else {
					jsonToRegisteredReference(json.xyz[i], robotId, referenceables,
						(value) => { trafo.elements[12 + i] = value }
					)
				}
			}
		}
		else if ("axisDistance" in json) {
			const axis = jsonToAxis(json.axisDistance.axis)
			if (typeof (json.axisDistance.distance) == "number") {
				trafo.setPosition(axis.clone().multiplyScalar(json.axisDistance.distance))
			}
			else {
				jsonToRegisteredReference(json.axisDistance.distance, robotId, referenceables,
					(value) => { trafo.setPosition(axis.clone().multiplyScalar(value)) }
				)
			}
		}

		// rotation
		const restorePosition = new THREE.Vector3(trafo.elements[12], trafo.elements[13], trafo.elements[14])
		if ("qwxyz" in json) {
			trafo.makeRotationFromQuaternion(jsonToQuaternion(json.qwxyz, "wxyz"))
			trafo.setPosition(restorePosition)
		}
		else if ("qxyzw" in json) {
			trafo.makeRotationFromQuaternion(jsonToQuaternion(json.qxyzw, "xyzw"))
			trafo.setPosition(restorePosition)
		}
		else if ("cols3x3" in json) { throw new Error("cols3x3 not yet implemented") }
		else if ("rows3x3" in json) { throw new Error("rows3x3 not yet implemented") }
		else if ("euler" in json) { throw new Error("euler not yet implemented") }
		else if ("axisAngle" in json) {
			const axis = jsonToAxis(json.axisAngle.axis)
			if (typeof (json.axisAngle.angle) == "number") {
				trafo.makeRotationAxis(axis, json.axisAngle.angle)
				trafo.setPosition(restorePosition)
			}
			else {
				jsonToRegisteredReference(json.axisAngle.angle, robotId, referenceables,
					(value) => {
						trafo.makeRotationAxis(axis, value)
						trafo.setPosition(restorePosition)
					}
				)
			}
		}
		else if ("rotvec" in json) { throw new Error("rotvec not yet implemented") }
	}

	return trafo
}

// todo: this function does not really belong into this file
export function jsonToRegisteredReference(json, robotId, referenceables, callback) {
	checkFields(json, [], ["joint", "trajectory", "index"])

	if (referenceables === null) {
		throw new Error("dynamic poses not supported in this place")
	}

	if ("joint" in json) {
		referenceables[`joint:${robotId}.${json.joint}`].addListener(callback)
	}
	else if ("trajectory" in json) {
		let ref = json.trajectory
		if ("index" in json) { ref += "." + json.index }
		referenceables[`trajectory:${ref}`].addListener(callback)
	}
	else {
		throw new Error("expected number, joint reference or trajectory reference")
	}
}

export function jsonToAxis(json) {
	if (json == "x") { return new THREE.Vector3(1, 0, 0) }
	else if (json == "y") { return new THREE.Vector3(0, 1, 0) }
	else if (json == "z") { return new THREE.Vector3(0, 0, 1) }
	else { return jsonToVector(json).normalize() }
}

export function jsonToVector(json) {
	console.assert(Array.isArray(json) && json.length == 3)
	return new THREE.Vector3(json[0], json[1], json[2])
}

export function jsonToQuaternion(json, order) {
	console.assert(Array.isArray(json) && json.length == 4)
	if (order === "xyzw") {
		return new THREE.Quaternion(json[0], json[1], json[2], json[3])
	}
	else if (order === "wxyz") {
		return new THREE.Quaternion(json[1], json[2], json[3], json[0])
	}
	else { throw new Error("invalid quaternion order") }
}
