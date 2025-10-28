
#pragma once

#include "stl.h"

namespace str{
	
	// load string from file
	inline std::string load(const std::string& filename, bool binary = false) {
		// https://stackoverflow.com/a/2912614
		std::ifstream ifs(
			filename,
			binary ?
			std::ios::in | std::ios::binary
			: std::ios::in
		);
		if (!ifs) {
			throw std::runtime_error(filename + " could not be read");
		}
		return std::string(
			(std::istreambuf_iterator<char>(ifs)),
			(std::istreambuf_iterator<char>()));
	}

	// save string to file
	inline void save(const std::string& filename, const std::string& content, bool binary = false) {
		std::ofstream ofs(
			filename,
			binary ?
			std::ios::out | std::ios::binary
			: std::ios::out
		);
		if (!ofs) {
			throw std::runtime_error(filename + " could not be written");
		}
		ofs << content;
	}

	// append string to file
	inline void append(const std::string& filename, const std::string& content) {
		std::ofstream ofs(filename, std::ofstream::out | std::ofstream::app);
		if (!ofs) {
			throw std::runtime_error(filename + " could not be written");
		}
		ofs << content;
	}

	// various string replace functions
	inline void replaceInplace(std::string& str, const std::string& search, const std::string& repl) {
		// https://stackoverflow.com/a/24315631
		size_t start_pos = 0;
		while ((start_pos = str.find(search, start_pos)) != std::string::npos) {
			str.replace(start_pos, search.length(), repl);
			start_pos += repl.length(); // Handles case where 'repl' is a substring of 'search'
		}
	}

	inline std::string replace(std::string str, const std::string& search, const std::string& repl) {
		replaceInplace(str, search, repl);
		return str;
	}

	inline void replaceInplace(std::string& str,
		const std::vector<std::pair<std::string, std::string>>& subs) {
		for (const auto& s : subs) {
			replaceInplace(str, s.first, s.second);
		}
	}

	inline std::string replace(std::string str, const std::vector<std::pair<std::string, std::string>>& subs) {
		replaceInplace(str, subs);
		return str;
	}

	inline std::string lower(std::string s) {
		for (char& c : s)
			c = tolower(c);
		return s;
	}

	inline std::string upper(std::string s) {
		for (char& c : s)
			c = toupper(c);
		return s;
	}

	inline std::vector<std::string> split(const std::string& s, char delimiter = ' ') {
		std::vector<std::string> tokens;
		std::string token;
		std::istringstream tokenStream(s);
		while (std::getline(tokenStream, token, delimiter)) {
			tokens.push_back(token);
		}
		return tokens;
	}

}