
#pragma once

#include <fstream>
#include <string>

inline std::string readFile(const std::string& filename) {
	std::ifstream ifs(filename);
	if (!ifs) {
		throw std::runtime_error(filename + " could not be read");
	}
	return std::string(
		(std::istreambuf_iterator<char>(ifs)),
		(std::istreambuf_iterator<char>()));
}

inline void writeFile(const std::string& filename, const std::string& content) {
	std::ofstream ofs(filename);
	if (!ofs) {
		throw std::runtime_error(filename + " could not be written");
	}
	ofs << content;
}
