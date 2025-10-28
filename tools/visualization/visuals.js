
import * as THREE from 'three'
import { GLTFLoader } from './GLTFLoader.js'

let cache = {}
export function loadVisuals(path) {
	if (path in cache) {
		return cache[path]
	}
	let result = new Promise((resolve, reject) => {
		const loader = new GLTFLoader()
		loader.load(
			path,
			function (gltf) {
				let visuals = {}
				gltf.scene.updateMatrixWorld(true)
				for (const child of gltf.scene.children) {
					child.matrixAutoUpdate = false
					child.userData.defaultWorldPose = child.matrixWorld.clone()
					visuals[child.name] = child
					child.name += "(visuals)"
				}
				resolve(visuals)
			},
			function (xhr) {
				// console.log((xhr.loaded / xhr.total * 100) + '% loaded')
			},
			function (error) {
				console.error('error loading ' + path)
				reject()
			}
		)
	})
	cache[path] = result
	return result
}
