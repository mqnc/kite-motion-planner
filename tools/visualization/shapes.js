
import * as THREE from 'three'
import { checkFields } from "./utils.js"
import { ConvexGeometry2 } from './ConvexGeometry2.js'
import { jsonToVector, jsonToAxis, jsonToDynamicMatrix } from './trafo.js'

const haloVertexShader = `
varying vec3 nml;
varying vec3 pos;

void main()
{
	pos = (modelMatrix * vec4(position, 1.0)).xyz;
	nml = (modelMatrix * vec4(normal, 0.0)).xyz;
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}
`
const haloFragmentShader = `
uniform vec3 glowColor;
varying vec3 pos;
varying vec3 nml;
void main()
{
	vec3 view = normalize(pos-cameraPosition);
	float intensity = pow(1.0 - abs(dot(normalize(nml), view)), 3.0);
	//gl_FragColor = vec4( glowColor, 0.9 * intensity + 0.1 );
	gl_FragColor = vec4( glowColor, intensity );
}
`


// const coreMaterial = new THREE.MeshStandardMaterial({
// 	roughness: 0.8,
// 	metalness: 0.2,
// })
let coreMaterial = new THREE.ShaderMaterial({
	uniforms: {
		glowColor: { value: new THREE.Color(0, 0.408, 0.459) },
	},
	vertexShader: haloVertexShader,
	fragmentShader: haloFragmentShader,
	transparent: true
});
export function setCoreMaterial(material) { coreMaterial = material }

let marginMaterial = new THREE.ShaderMaterial({
	uniforms: {
		glowColor: { value: new THREE.Color(0.584, 0.059, 0.329) },
	},
	vertexShader: haloVertexShader,
	fragmentShader: haloFragmentShader,
	side: THREE.BackSide,
	transparent: true
});
export function setMarginMaterial(material) { marginMaterial = material }

let numCircleCorners = 40
let icoDetail = Math.floor(numCircleCorners / 5 - 1)
let sphereTemplateGeometry = new THREE.IcosahedronGeometry(1, icoDetail)
let sphereTemplate = new THREE.Mesh(sphereTemplateGeometry)
let origin = { xyz: [0, 0, 0], qxyzw: [0, 0, 0, 1] }

const commonShapeKeys = ["margin", "collides", "stickyForce", "visuals"]

export function jsonToShapeObject(json, shapeId) {
	checkFields(json, ["kind"], [...commonShapeKeys,
		"center", "radius", "pose", "size", "height", "axis", "points", "vertices", "faces"])

	switch (json.kind) {
		case "box": return jsonToBoxObject(json, shapeId)
		case "sphere": return jsonToSphereObject(json, shapeId)
		case "cylinder": return jsonToCylinderObject(json, shapeId)
		case "orangeNet": return jsonToOrangeNetObject(json, shapeId)
		case "mesh": // return parseMesh(json)
		default: throw new Error("unhandled shape type: " + json.kind)
	}
}

function makeOrangeNetGeometry(points, radius) {
	let helper = new THREE.Group()
	for (const p of points) {
		let s = sphereTemplate.clone()
		s.position.set(p[0], p[1], p[2])
		const r = radius
		s.scale.set(r, r, r)
		helper.add(s)
	}
	let geometry = new ConvexGeometry2(helper)
	geometry.computeVertexNormals()
	return geometry
}

function jsonToBoxObject(json, shapeId) {
	checkFields(json, ["kind", "size"], [...commonShapeKeys, "pose"])

	let result = new THREE.Group()
	result.name = shapeId
	let coreGeometry = new THREE.BoxGeometry(json.size[0], json.size[1], json.size[2])
	let coreMesh = new THREE.Mesh(coreGeometry, coreMaterial.clone())
	coreMesh.name = "core"
	result.add(coreMesh)
	let margin = json.margin ?? 0
	let marginGeometry
	if (margin == 0) {
		marginGeometry = coreGeometry.clone()
	}
	else {
		let [w2, h2, d2] = [json.size[0] / 2, json.size[1] / 2, json.size[2] / 2]
		marginGeometry = makeOrangeNetGeometry([
			[-w2, -h2, -d2], [-w2, -h2, d2], [-w2, h2, -d2], [-w2, h2, d2],
			[w2, -h2, -d2], [w2, -h2, d2], [w2, h2, -d2], [w2, h2, d2]
		], margin)
	}
	let marginMesh = new THREE.Mesh(marginGeometry, marginMaterial.clone())
	marginMesh.name = "margin"
	result.add(marginMesh)
	result.matrixAutoUpdate = false
	result.matrix = jsonToDynamicMatrix(json.pose ?? origin, "", null)
	return result
}

function jsonToSphereObject(json, shapeId) {
	checkFields(json, ["kind", "radius"], [...commonShapeKeys, "center"])

	let result = new THREE.Group()
	result.name = shapeId
	let coreMesh = sphereTemplate.clone()
	coreMesh.name = "core"
	coreMesh.material = coreMaterial.clone()
	let r = json.radius
	coreMesh.scale.set(r, r, r)
	result.add(coreMesh)
	let marginMesh = sphereTemplate.clone()
	marginMesh.name = "margin"
	marginMesh.material = marginMaterial.clone()
	r = json.radius + json.margin
	marginMesh.scale.set(r, r, r)
	result.add(marginMesh)
	result.position.copy(jsonToVector(json.center ?? [0, 0, 0]))
	return result
}

function jsonToOrangeNetObject(json, shapeId) {
	checkFields(json, ["kind", "points", "radius"], commonShapeKeys)

	let result = new THREE.Group()
	result.name = shapeId
	let coreGeometry = makeOrangeNetGeometry(json.points, json.radius)
	let coreMesh = new THREE.Mesh(coreGeometry, coreMaterial.clone())
	coreMesh.name = "core"
	result.add(coreMesh)
	let margin = json.margin ?? 0
	let marginGeometry = makeOrangeNetGeometry(json.points, json.radius + margin)
	let marginMesh = new THREE.Mesh(marginGeometry, marginMaterial.clone())
	marginMesh.name = "margin"
	result.add(marginMesh)
	return result
}

function jsonToCylinderObject(json, shapeId) {
	checkFields(json, ["kind", "radius", "height", "axis"], [...commonShapeKeys, "center"])

	let poseHelper = new THREE.Group()
	poseHelper.name = "helper"
	let coreGeometry = new THREE.CylinderGeometry(json.radius, json.radius, json.height, numCircleCorners)
	let coreMesh = new THREE.Mesh(coreGeometry, coreMaterial.clone())
	coreMesh.name = "core"
	coreMesh.rotation.x = Math.PI / 2
	poseHelper.add(coreMesh)
	let margin = json.margin ?? 0
	if (margin == 0) {
		let marginGeometry = coreGeometry.clone()
		let marginMesh = new THREE.Mesh(marginGeometry, marginMaterial.clone())
		marginMesh.name = "margin"
		marginMesh.rotation.x = Math.PI / 2
		poseHelper.add(marginMesh)
	}
	else {
		let helper = new THREE.Group()
		let torusGeometry = new THREE.TorusGeometry(json.radius, margin, numCircleCorners, numCircleCorners)
		let torus1 = new THREE.Mesh(torusGeometry)
		let torus2 = torus1.clone()
		torus1.position.setZ(json.height / 2)
		torus2.position.setZ(-json.height / 2)
		helper.add(torus1)
		helper.add(torus2)
		let marginGeometry = new ConvexGeometry2(helper)
		marginGeometry.computeVertexNormals()
		let marginMesh = new THREE.Mesh(marginGeometry, marginMaterial.clone())
		marginMesh.name = "margin"
		poseHelper.add(marginMesh)
	}


	poseHelper.matrixAutoUpdate = false
	poseHelper.matrix.multiply(new THREE.Matrix4().lookAt(
		new THREE.Vector3(0, 0, 0),
		jsonToAxis(json.axis),
		new THREE.Vector3(0, 0, 1)
	)).setPosition(jsonToVector(json.center ?? [0, 0, 0]))

	let result = new THREE.Group()
	result.name = shapeId
	result.add(poseHelper)

	return result
}
