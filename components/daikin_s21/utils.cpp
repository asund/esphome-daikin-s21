#include "utils.h"

bool uint8_starts_with_str(const std::vector<uint8_t>& vec, const char* str) {
    if (vec.size() < strlen(str)) {
        return false;
    }
  
    for (size_t i = 0; i < strlen(str); i++)
    {
      if(vec[i] == static_cast<uint8_t>(str[i])) {
        continue;
      } else {
        return false;
      }
    }
    
    return true;
  }

bool is_little_endian() {
    union {
      uint32_t i;
      uint8_t c[4];
    } test = {0x01020304};
    
    // If we're little-endian, the least significant byte (0x04) will be at the lowest address
    // If we're big-endian, the most significant byte (0x01) will be at the lowest address
    return test.c[0] == 0x04;
}

// Adapated from ESPHome UART debugger
std::string hex_repr(uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (i > 0)
      res += ':';
    sprintf(buf, "%02X", bytes[i]);
    res += buf;
  }
  return res;
}

std::string hex_repr(std::vector<uint8_t> &bytes) {
  return hex_repr(&bytes[0], bytes.size());
}

std::string bin_repr(uint8_t *bytes, size_t len, bool little_endian) {
  std::string res;
  char buf[9]; // 8 bits + null terminator
  for (size_t i = 0; i < len; i++) {
    if (i > 0)
      res += ' ';
    if (little_endian) {
      // Little-endian: LSB (bit 0) first
      for (int j = 0; j <= 7; j++) {
        buf[j] = ((bytes[i] & (1 << j)) ? '1' : '0');
      }
    } else {
      // Big-endian: MSB (bit 7) first (default)
      for (int j = 7; j >= 0; j--) {
        buf[7-j] = ((bytes[i] & (1 << j)) ? '1' : '0');
      }
    }
    buf[8] = '\0';
    res += buf;
  }
  return res;
}

std::string bin_repr(std::vector<uint8_t> &bytes, bool little_endian) {
  return bin_repr(&bytes[0], bytes.size(), little_endian);
}

std::string bin_repr(std::vector<uint8_t> &bytes) {
  return bin_repr(&bytes[0], bytes.size(), true) + " defaulted to little-endian";
}

// Adapated from ESPHome UART debugger
std::string str_repr(uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] == 7) {
      res += "\\a";
    } else if (bytes[i] == 8) {
      res += "\\b";
    } else if (bytes[i] == 9) {
      res += "\\t";
    } else if (bytes[i] == 10) {
      res += "\\n";
    } else if (bytes[i] == 11) {
      res += "\\v";
    } else if (bytes[i] == 12) {
      res += "\\f";
    } else if (bytes[i] == 13) {
      res += "\\r";
    } else if (bytes[i] == 27) {
      res += "\\e";
    } else if (bytes[i] == 34) {
      res += "\\\"";
    } else if (bytes[i] == 39) {
      res += "\\'";
    } else if (bytes[i] == 92) {
      res += "\\\\";
    } else if (bytes[i] < 32 || bytes[i] > 127) {
      sprintf(buf, "\\x%02X", bytes[i]);
      res += buf;
    } else {
      res += bytes[i];
    }
  }
  return res;
}

std::string str_repr(std::vector<uint8_t> &bytes) {
  return str_repr(&bytes[0], bytes.size());
}
