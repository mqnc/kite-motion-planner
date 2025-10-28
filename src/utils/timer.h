
#pragma once

#include <chrono>
#include <ostream>
#include <string>
#include <iomanip>

class Timer {
	bool running = false;
	std::chrono::time_point<std::chrono::high_resolution_clock> tStart;
	std::chrono::duration<double> tSum = std::chrono::duration<double>::zero();

public:
	void run() {
		tStart = std::chrono::high_resolution_clock::now();
		if (running) { throw std::runtime_error("timer already running"); }
		running = true;
	}

	void pause() {
		auto now = std::chrono::high_resolution_clock::now();
		if (!running) { throw std::runtime_error("timer was not running"); }
		tSum += now - tStart;
		running = false;
	}

	void reset() {
		tSum = std::chrono::duration<double>::zero();
		running = false;
	}

	Timer(bool start = true) {
		if (start) { run(); }
	}

	std::chrono::duration<double> elapsed() const {
		auto now = std::chrono::high_resolution_clock::now();
		if (running) { return tSum + (now - tStart); }
		else { return tSum; }
	}

	std::string formatElapsed() const {
		using namespace std::chrono;
		auto delta = elapsed();

		auto h = duration_cast<hours>(delta);
		delta -= h;
		auto m = duration_cast<minutes>(delta);
		delta -= m;
		auto s = duration_cast<seconds>(delta);
		delta -= s;
		auto ms = duration_cast<milliseconds>(delta);
		delta -= ms;
		auto us = duration_cast<microseconds>(delta);

		std::ostringstream formatted_duration;
		formatted_duration << std::setfill('0') <<
			std::setw(2) << h.count() << ':' <<
			std::setw(2) << m.count() << ':' <<
			std::setw(2) << s.count() << '.' <<
			std::setw(3) << ms.count() << '\'' <<
			std::setw(3) << us.count();

		return formatted_duration.str();
	}

	double s() const {
		return elapsed().count();
	}
};

inline std::ostream& operator<<(std::ostream& os, const Timer& t) {
	os << t.formatElapsed();
	return os;
}
