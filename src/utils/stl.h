
#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <tuple>
#include <random>
#include <sstream>
#include <set>
#include <stdexcept>
#include <stdlib.h>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <valarray>
#include <vector>

// this is arguably bad practice but it unclutters the code considerably
using std::cout;
using std::string;
using std::literals::string_literals::operator""s;
using std::stringstream;
using std::to_string;

using std::array;
using std::deque;
using std::list;
using std::vector;
using std::valarray;
using std::variant;
using std::function;
using std::optional;
using std::pair;
using std::set;

// see example: https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts> struct overload: Ts... { using Ts::operator()...; };
template <class... Ts> overload(Ts...) -> overload<Ts...>;

template <typename T>
using Dict = std::unordered_map<std::string, T>;

using Waypoint = valarray<double>;
using Path = valarray<Waypoint>;

using std::unordered_set;

using std::unique_ptr;
using std::make_unique;
// using std::move; // clang warns about this
using std::shared_ptr;
using std::make_shared;
using std::weak_ptr;

using std::runtime_error;

const auto NaN = std::nan("");
using std::isnan;
const auto inf = std::numeric_limits<double>::infinity();
using std::isinf;
using std::abs;
using std::min;
using std::max;
using std::begin;
using std::end;

inline constexpr double operator""_deg(long double deg) {
	return deg * std::numbers::pi_v<long double> / 180.0;
}

inline constexpr double operator"" _deg(unsigned long long deg) {
    return deg * std::numbers::pi_v<long double> / 180.0;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const valarray<T>& va) {
	os << "[";
	string sep = "";
	for (const auto& v: va) {
		os << sep << v;
		sep = ", ";
	}
	os << "]";
	return os;
}

inline string zeroPad(size_t number, size_t length) {
	string nString = std::to_string(number);
	return std::string(length - std::min(length, nString.length()), '0') + nString;
}
