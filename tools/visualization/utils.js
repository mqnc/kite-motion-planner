
export function checkFields(obj, requiredKeys, optionalKeys = []) {
	const objKeys = Object.keys(obj)

	const missingRequiredFields = requiredKeys.filter(key => !objKeys.includes(key))
	if (missingRequiredFields.length > 0) {
		console.error(obj, "misses required", missingRequiredFields)
		throw new Error("see above")
	}

	if (optionalKeys !== "*") {
		const unexpectedFields = objKeys.filter(key => !requiredKeys.includes(key) && !optionalKeys.includes(key))
		if (unexpectedFields.length > 0) {
			console.error(obj, "contains unexpected", unexpectedFields)
			throw new Error("see above")
		}
	}
}


export class Referenceable {
	constructor(updateFilter = value => value) {
		this.listeners = []
		this.updateFilter = updateFilter
		this.cache = null
	}
	addListener(callback) {
		this.listeners.push(callback)
		if (this.cache !== null) {
			callback(this.cache)
		}
	}
	update(value) {
		const filteredValue = this.updateFilter(value)
		for (const listener of this.listeners) {
			listener(filteredValue)
		}
		this.cache = filteredValue
	}
}
