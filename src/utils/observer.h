
// Implementation of the Observer pattern
// https://en.wikipedia.org/wiki/Observer_pattern

#pragma once

#include <vector>
#include <functional>

using namespace std;

template <typename T>
class Observer;

template <typename T>
class Observable {
	T cache;
	std::vector<Observer<T>*> observers;

	void attach(Observer<T>* p) {
		observers.push_back(p);
	}

	void remove(Observer<T>* p) {
		observers.erase(std::find(observers.begin(), observers.end(), p));
	}

public:

	template <typename... Ts>
	Observable(Ts&&... args): cache(std::forward<Ts>(args)...) {}

	Observable(const Observable&) = delete;

	Observable(Observable&& other) = delete;

	template <typename... Ts>
	void update(Ts&&... args) {
		bool changed = cache.update(std::forward<Ts>(args)...);
		if (changed) {
			for (auto o: observers) {
				o->update(cache);
			}
		}
	}

	const T& getCached() const {
		return cache;
	}

	~Observable() {
		for (auto o: observers) {
			o->pSubject = nullptr;
		}
	}

	friend class Observer<T>;
};

template <typename T>
class Observer {
	Observable<T>* pSubject = nullptr;
	std::function<void(const T&)> callback;

	void update(const T& data) {
		if (callback) { callback(data); }
	}

public:

	Observer() {}

	Observer(Observable<T>& subject, std::function<void(const T&)> callback) {
		observe(subject);
		onUpdate(callback);
	}

	Observer(const Observer& other) = delete;

	Observer(Observer&& other) = delete;

	void observe(Observable<T>& subject) {
		assert(pSubject == nullptr);
		pSubject = &subject;
		pSubject->attach(this);
		update(subject.cache);
	}

	void onUpdate(std::function<void(const T&)> callback) {
		this->callback = callback;
	}

	const T& getCached() const {
		return pSubject->cache;
	}

	~Observer() {
		if (pSubject) {
			pSubject->remove(this);
		}
	}

	friend class Observable<T>;
};
