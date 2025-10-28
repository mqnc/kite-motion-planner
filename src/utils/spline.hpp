
template<typename T>
class Spline {

	array<T, 6> calcQuinticCoeffs(
		double t0, double t1,
		T y0, T dy0, T ddy0,
		T y1, T dy1, T ddy1
	) {
		// normalize to t=0..1
		const double t01 = t1 - t0;
		const double tt01 = t01 * t01;
		dy0 = t01 * dy0;
		dy1 = t01 * dy1;
		ddy0 = tt01 * ddy0;
		ddy1 = tt01 * ddy1;

		//   y    =   c5t^5 +   c4t^4 +  c3t^3 +  c2t^2 + c1t + c0
		//   dy   =  5c5t^4 +  4c4t^3 + 3c3t^2 + 2c2t   + c1
		//   ddy  = 20c5t^3 + 12c4t^2 + 6c3t   + 2c2

		//   ⎛ y0   ⎞     ⎛  0  0  0  0  0  1 ⎞ ⎛ c5 ⎞
		//   ⎜ dy0  ⎟     ⎜  0  0  0  0  1  0 ⎟ ⎜ c4 ⎟
		//   ⎜ ddy0 ⎟ ─── ⎜  0  0  0  2  0  0 ⎟ ⎜ c3 ⎟
		//   ⎜ y1   ⎟ ─── ⎜  1  1  1  1  1  1 ⎟ ⎜ c2 ⎟
		//   ⎜ dy1  ⎟     ⎜  5  4  3  2  1  0 ⎟ ⎜ c1 ⎟
		//   ⎝ ddy1 ⎠     ⎝ 20 12  6  2  0  0 ⎠ ⎝ c0 ⎠

		//   ⎛ c5 ⎞     ⎛ - 6  -3 -0.5   6  -3   0.5 ⎞ ⎛ y0   ⎞
		//   ⎜ c4 ⎟     ⎜  15   8  1.5 -15   7  -1   ⎟ ⎜ dy0  ⎟
		//   ⎜ c3 ⎟ ─── ⎜ -10  -6 -1.5  10  -4   0.5 ⎟ ⎜ ddy0 ⎟
		//   ⎜ c2 ⎟ ─── ⎜   0   0  0.5   0   0   0   ⎟ ⎜ y1   ⎟
		//   ⎜ c1 ⎟     ⎜   0   1    0   0   0   0   ⎟ ⎜ dy1  ⎟
		//   ⎝ c0 ⎠     ⎝   1   0    0   0   0   0   ⎠ ⎝ ddy1 ⎠

		const T c5 = -6.0 * y0 - 3.0 * dy0 - 0.5 * ddy0 + 6.0 * y1 - 3.0 * dy1 + 0.5 * ddy1;
		const T c4 = 15.0 * y0 + 8.0 * dy0 + 1.5 * ddy0 - 15.0 * y1 + 7.0 * dy1 - 1.0 * ddy1;
		const T c3 = -10.0 * y0 - 6.0 * dy0 - 1.5 * ddy0 + 10.0 * y1 - 4.0 * dy1 + 0.5 * ddy1;
		const T c2 = 0.5 * ddy0;
		const T c1 = dy0;
		const T c0 = y0;

		return array<T, 6> {c0, c1, c2, c3, c4, c5};
	}

public:
	const double t0, t1;
	const array<T, 6> c;

	Spline(
		double t0, double t1,
		T y0, T dy0, T ddy0,
		T y1, T dy1, T ddy1
	) :
		t0{ t0 },
		t1{ t1 },
		c{ calcQuinticCoeffs(t0, t1, y0, dy0, ddy0, y1, dy1, ddy1) }
	{}

	Spline(
		double t0, double t1,
		array<T, 6> c
	) :
		t0{ t0 },
		t1{ t1 },
		c{ c }
	{}

	T operator()(double t) const {
		double q = (t - t0) / (t1 - t0);
		return ((((c[5] * q + c[4]) * q + c[3]) * q + c[2]) * q + c[1]) * q + c[0];
	}

	vector<T> operator()(const vector<double>& ts) const {
		vector<T> result;
		result.reserve(ts.size());
		for (const auto& t : ts) {
			result.push_back(this->operator()(t));
		}
		return result;
	}

	valarray<T> operator()(const valarray<double>& ts) const {
		valarray<T> result(ts.size());
		for (size_t i = 0; i < ts.size(); i++) {
			result[i]=this->operator()(ts[i]);
		}
		return result;
	}
};

template<typename T>
class ConsecutiveSplines {
	vector<Spline<T>> segments;
	double totalDuration = 0;

public:
	ConsecutiveSplines(size_t allocateNumSegments = 0) {
		segments.reserve(allocateNumSegments);
	}

	class Iterator {
		const ConsecutiveSplines<T>& curve;
		double t = 0;
		size_t seg = 0;

	public:

		Iterator(
			const ConsecutiveSplines<T>& curve,
			double t = 0,
			size_t seg = 0
		) :
			curve{ curve },
			t{ t },
			seg{ seg }
		{}

		T operator*() const {
			return curve.segments[seg](t);
		}

		void operator+=(double dt) {
			assert(dt >= 0);
			t += dt;
			if (t > curve.totalDuration) { t = std::nextafter(curve.totalDuration, inf); }
			while (curve.segments[seg].t1 <= t) {
				seg++;
			}
		}

		Iterator operator+(double dt) const {
			Iterator result{ curve, t, seg };
			result += dt;
			return result;
		}

		bool operator==(const Iterator& other) {
			return &curve == &(other.curve) && t == other.t;
		}

		bool operator!=(const Iterator& other) {
			return !(*this == other);
		}
	};

	double getTotalDuration() const { return totalDuration; }

	void append(
		double duration,
		T y0, T dy0, T ddy0,
		T y1, T dy1, T ddy1
	) {
		segments.emplace_back(
			totalDuration, totalDuration + duration,
			y0, dy0, ddy0,
			y1, dy1, ddy1
		);
		totalDuration += duration;
	}

	void append(
		const Spline<T>& segment
	) {
		double duration = segment.t1 - segment.t0;
		segments.emplace_back(
			totalDuration, totalDuration + duration,
			segment.c
		);
		totalDuration += duration;
	}

	Iterator begin() const {
		return Iterator(*this, 0, 0);
	}

	// end() is behind the last element in std iterators
	Iterator end() const {
		return Iterator(*this, std::nextafter(totalDuration, inf), segments.size());
	}
};
