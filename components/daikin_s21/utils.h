#pragma once

#include <vector>
#include <cstdint>
#include <string.h>
#include <string>

bool uint8_starts_with_str(const std::vector<uint8_t>& vec, const char* str);
bool is_little_endian();
std::string hex_repr(uint8_t *bytes, size_t len);
std::string hex_repr(std::vector<uint8_t> &bytes);
std::string bin_repr(uint8_t *bytes, size_t len, bool little_endian);
std::string bin_repr(std::vector<uint8_t> &bytes, bool little_endian);
std::string bin_repr(std::vector<uint8_t> &bytes);
std::string str_repr(uint8_t *bytes, size_t len);
std::string str_repr(std::vector<uint8_t> &bytes);
