
// stolen from https://www.npmjs.com/package/idb-keyval
// lot of effort to reopen the most recent file

function promisifyRequest(request) {
	return new Promise((resolve, reject) => {
		request.oncomplete = request.onsuccess = () => resolve(request.result);
		request.onabort = request.onerror = () => reject(request.error);
	});
}

function createStore(dbName, storeName) {
	const request = indexedDB.open(dbName);
	request.onupgradeneeded = () => request.result.createObjectStore(storeName);
	const dbp = promisifyRequest(request);
	return (txMode, callback) => dbp.then((db) => callback(db.transaction(storeName, txMode).objectStore(storeName)));
}

let defaultGetStoreFunc;
function defaultGetStore() {
	if (!defaultGetStoreFunc) {
		defaultGetStoreFunc = createStore('keyval-store', 'keyval');
	}
	return defaultGetStoreFunc;
}

function idbLoad(key, customStore = defaultGetStore()) {
	return customStore('readonly', (store) => promisifyRequest(store.get(key)));
}

function idbStore(key, value, customStore = defaultGetStore()) {
	return customStore('readwrite', (store) => {
		store.put(value, key);
		return promisifyRequest(store.transaction);
	});
}

function makeKey(domElement){
	return 'lastOpened-' + domElement.tagName + "-" + domElement.id;
}

async function handleFile(domElement, file) {
	await idbStore(makeKey(domElement), file);
	const reader = new FileReader();
	reader.onload = function (e) {
		domElement.dispatchEvent(
			new CustomEvent("handlefilecontent", { detail: e.target.result })
		);
	};
	reader.readAsText(file);
}

function dropHandler(ev) {
	ev.preventDefault();

	if (ev.dataTransfer.items) {
		[...ev.dataTransfer.items].forEach((item, i) => {
			if (item.kind === "file") {
				const file = item.getAsFile();
				handleFile(ev.currentTarget, file);
			}
		});
	} else {
		[...ev.dataTransfer.files].forEach((file, i) => {
			handleFile(ev.currentTarget, file);
		});
	}
}

function dragOverHandler(ev) {
	ev.preventDefault();
}

async function checkLastOpened(domElement) {
	try {
		let file = await idbLoad(makeKey(domElement));
		if (file) {
			handleFile(domElement, file)
		}
	} catch (error) {
		console.error(error.name, error.message);
	}
}

export function dragDropInitialize(domElement, callback) {
	domElement.addEventListener("drop", dropHandler)
	domElement.addEventListener("dragover", dragOverHandler)
	domElement.addEventListener("handlefilecontent", (ev) => { callback(ev.detail) })
	checkLastOpened(domElement)
}