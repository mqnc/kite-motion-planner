
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <tuple>
#include <algorithm>
#include <functional>
#include "timer.h"

class Profiling {
	inline static Timer programTimer {false};
	inline static std::unordered_map<std::string, Timer> timers;

	struct OnScopeExit {
		std::function<void(void)> cb;
		~OnScopeExit() { cb(); }
	};

public:

	static void run(std::string name) {
		if (!timers.contains(name)) {
			timers.try_emplace(name, false);
		}
		timers.at(name).run();
	}

	static void pause(std::string name) {
		timers.at(name).pause();
	}

	static OnScopeExit scope(std::string name) {
		Profiling::run(name);
		return OnScopeExit {[name]() { Profiling::pause(name); }};
	}

	Profiling() {
		programTimer.run();
	}

	~Profiling() {
		std::vector<std::pair<std::string, Timer>> timerVector(timers.begin(), timers.end());

		std::sort(timerVector.begin(), timerVector.end(),
			[](const auto& a, const auto& b) {
				return a.second.s() > b.second.s();
			});

		programTimer.pause();
		std::cout << "\n" << programTimer << " (100.00 %) program runtime\n";
		std::cout << "------------------------------------------\n";

		for (const auto& [name, timer]: timerVector) {
			std::cout << timer << " (" << (100 * timer.s() / programTimer.s()) << " %) " << name << "\n";
		}
	}
};

inline Profiling profilerSingleton;

#define PROFILE_RUN(NAME) Profiling::run(#NAME)
#define PROFILE_PAUSE(NAME) Profiling::pause(#NAME)
#define PROFILE_SCOPE(NAME) auto profiling___scope = Profiling::scope(#NAME)