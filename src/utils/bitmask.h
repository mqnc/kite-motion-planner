
#pragma once

#include <cstdint>
#include <vector>

class BitMask {
private:
	std::vector<uint64_t> chunks;
	size_t size = 0;

	void expandToFit(std::size_t index) {
		std::size_t requiredSize = (index / 64) + 1;
		if (chunks.size() < requiredSize) {
			chunks.resize(requiredSize, 0);
		}
	}

public:
	void set(std::size_t index, bool value = true) {
		expandToFit(index);
		size = max(size, index + 1);
		if (value) {
			chunks[index / 64] |= (1ULL << (index % 64));
		} else {
			chunks[index / 64] &= ~(1ULL << (index % 64));
		}
	}

	void clear() {
		chunks.clear();
		size = 0;
	}

	bool operator&(const BitMask& other) const {
		std::size_t commonSize = std::min(chunks.size(), other.chunks.size());
		for (std::size_t i = 0; i < commonSize; ++i) {
			if ((chunks[i] & other.chunks[i]) != 0) {
				return true;
			}
		}
		return false;
	}

	std::string toString() const {
		std::string result;
		for (std::size_t index = 0; index < size; ++index) {
			std::size_t chunkIndex = index / 64;
			std::size_t bitIndex = index % 64;
			bool bitValue = (chunks[chunkIndex] >> bitIndex) & 1;
			result += bitValue ? '1' : '0';
		}
		return result;
	}

};
