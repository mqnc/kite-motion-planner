#include <array>
#include <vector>

using std::size_t;

std::vector<size_t> divideAndConquerShuffle(size_t n) {

	if (n == 0) { return {}; }
	if (n == 1) { return {0}; }
	if (n == 2) { return {0, 1}; }

	std::vector<size_t> result;
	result.reserve(n);

	result.push_back(0);
	result.push_back(n - 1);

	std::queue<std::array<size_t, 2>> tbd;

	tbd.push({1, n - 2});

	while (tbd.size() > 0) {
		const auto [from, to] = tbd.front();
		tbd.pop();
		size_t mid = from + (to - from) / 2;
		result.push_back(mid);
		if (from < mid) {
			tbd.push({from, mid - 1});
		}
		if (mid < to) {
			tbd.push({mid + 1, to});
		}
	}

	return result;
}
