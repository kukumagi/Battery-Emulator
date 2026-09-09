#include "utils.h"

#include <fstream>

namespace fs = std::filesystem;

bool ends_with(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::vector<std::string> split(const std::string& text, char sep) {
  std::vector<std::string> tokens;
  std::size_t start = 0, end = 0;
  while ((end = text.find(sep, start)) != std::string::npos) {
    tokens.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  return tokens;
}

void print_frame(const CAN_frame& frame) {
  std::cout << "ID: " << std::hex << frame.ID << ", DLC: " << (int)frame.DLC << ", Data: ";
  for (int i = 0; i < frame.DLC; ++i) {
    std::cout << std::hex << (int)frame.data.u8[i] << " ";
  }
  std::cout << std::dec << "\n";
}

std::string snake_case_to_camel_case(const std::string& str) {
  std::string result;
  bool toUpper = false;
  for (char ch : str) {
    if (ch == '_') {
      toUpper = true;
    } else if (ch < '0' || (ch > '9' && ch < 'A') || (ch > 'Z' && ch < 'a') || ch > 'z') {
      // skip non-alphanumeric characters
      toUpper = true;
    } else {
      if (toUpper) {
        result += toupper(ch);
        toUpper = false;
      } else {
        result += ch;
      }
    }
  }
  return result;
}

CAN_frame parse_can_log_line(const std::string& logLine) {
  std::stringstream ss(logLine);
  CAN_frame frame = {};
  char dummy;

  double timestamp;
  std::string interfaceName;

  // timestamp and interface name are parsed but not used
  ss >> dummy >> timestamp >> dummy;
  ss >> interfaceName;

  // parse hexadecimal CAN ID
  ss >> std::hex >> frame.ID;
  if (ss.fail()) {
    throw std::runtime_error("Invalid format: Failed to parse CAN ID.");
  }
  // check whether the ID is in the extended range
  frame.ext_ID = (frame.ID > 0x7FF);

  // parse the data length
  int dlc_val;
  ss >> dummy;  // Consume '['
  if (ss.fail() || dummy != '[') {
    throw std::runtime_error("Invalid format: Missing opening bracket for data length.");
  }
  ss >> dlc_val;
  frame.DLC = static_cast<uint8_t>(dlc_val);
  ss >> dummy;  // Consume ']'
  if (ss.fail() || dummy != ']') {
    throw std::runtime_error("Invalid format: Missing closing bracket for data length.");
  }
  // crudely assume CAN FD if DLC > 8
  frame.FD = (frame.DLC > 8);

  // parse the actual data bytes
  unsigned int byte;
  for (int i = 0; i < frame.DLC; ++i) {
    ss >> std::hex >> byte;
    if (ss.fail()) {
      throw std::runtime_error("Fewer data bytes than specified by data length.");
    }
    frame.data.u8[i] = static_cast<uint8_t>(byte);
  }

  return frame;
}

namespace {

bool is_hex_digit(char ch) {
  return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F');
}

std::string trim(const std::string& input) {
  const auto start = input.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(start, end - start + 1);
}

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    out.push_back(field);
  }
  return out;
}

bool try_parse_hex_byte(const std::string& token, uint8_t& value) {
  if (token.empty() || token.size() > 2) {
    return false;
  }
  for (char ch : token) {
    if (!is_hex_digit(ch)) {
      return false;
    }
  }
  value = static_cast<uint8_t>(std::stoul(token, nullptr, 16));
  return true;
}

std::vector<CAN_frame> parse_csv_can_log_file(const fs::path& filePath) {
  std::ifstream logFile(filePath);
  if (!logFile.is_open()) {
    return {};
  }

  std::vector<CAN_frame> frames;
  std::string line;
  while (std::getline(logFile, line)) {
    if (line.empty()) {
      continue;
    }
    if (line.rfind("Time Stamp", 0) == 0) {
      continue;
    }

    auto cols = split_csv_line(line);
    if (cols.size() < 14) {
      continue;
    }
    try {
      CAN_frame frame = {};
      frame.ID = static_cast<uint16_t>(std::stoul(cols[1], nullptr, 16));
      frame.ext_ID = (cols[2] == "true");
      const uint8_t dlc = static_cast<uint8_t>(std::stoi(cols[5]));
      frame.DLC = dlc;
      frame.FD = (dlc > 8);
      for (uint8_t i = 0; i < dlc && i < 8; ++i) {
        const std::string token = cols[6 + i];
        if (token.empty()) {
          frame.data.u8[i] = 0;
          continue;
        }
        uint8_t value = 0;
        if (!try_parse_hex_byte(token, value)) {
          throw std::runtime_error("invalid CSV byte");
        }
        frame.data.u8[i] = value;
      }
      frames.push_back(frame);
    } catch (const std::exception&) {
      // Ignore malformed rows; the log sets are for validation, not generic CAN parsing.
    }
  }
  return frames;
}

std::vector<CAN_frame> parse_asc_can_log_file(const fs::path& filePath) {
  std::ifstream logFile(filePath);
  if (!logFile.is_open()) {
    return {};
  }

  std::vector<CAN_frame> frames;
  std::string line;
  while (std::getline(logFile, line)) {
    if (line.empty()) {
      continue;
    }
    if (line.find("Start of measurement") != std::string::npos || line.find("Begin Triggerblock") != std::string::npos ||
        line.find("version 8.5.0") != std::string::npos || line.find("Time Stamp") != std::string::npos) {
      continue;
    }

    std::istringstream iss(line);
    double timestamp = 0.0;
    std::string slot;
    std::string can_id_token;
    std::string dir;
    std::string mode;
    std::string dlc_token;
    std::string token;
    if (!(iss >> timestamp >> slot >> can_id_token >> dir >> mode >> dlc_token)) {
      continue;
    }

    if (dir != "Rx" && dir != "Tx") {
      continue;
    }

    try {
      const uint16_t can_id = static_cast<uint16_t>(std::stoul(can_id_token, nullptr, 16));
      const uint8_t dlc = static_cast<uint8_t>(std::stoi(dlc_token));
      CAN_frame frame = {};
      frame.ID = can_id;
      frame.ext_ID = false;
      frame.DLC = dlc;
      frame.FD = (dlc > 8);
      uint8_t byte_count = 0;
      while (iss >> token && byte_count < dlc) {
        if (token.find("Length") == 0 || token.find("ID") == 0) {
          break;
        }
        uint8_t value = 0;
        if (!try_parse_hex_byte(token, value)) {
          break;
        }
        frame.data.u8[byte_count++] = value;
      }
      if (byte_count == dlc) {
        frames.push_back(frame);
      }
    } catch (const std::exception&) {
      // Ignore malformed rows.
    }
  }
  return frames;
}

}  // namespace

std::vector<CAN_frame> parse_can_log_file(const fs::path& filePath) {
  const std::string extension = filePath.extension().string();
  if (extension == ".csv") {
    return parse_csv_can_log_file(filePath);
  }
  if (extension == ".asc") {
    return parse_asc_can_log_file(filePath);
  }

  std::ifstream logFile(filePath);
  if (!logFile.is_open()) {
    std::cerr << "Error: Could not open file " << filePath << std::endl;
    return {};
  }

  std::vector<CAN_frame> frames;
  std::string line;
  int lineNumber = 0;

  while (std::getline(logFile, line)) {
    lineNumber++;
    if (line.empty()) {
      continue;
    }

    if (line[0] == '#' || line[0] == ';') {
      continue;
    }

    try {
      frames.push_back(parse_can_log_line(line));
    } catch (const std::runtime_error& e) {
      std::cerr << "Warning: Skipping malformed line " << lineNumber << " in " << filePath.filename()
                << ". Reason: " << e.what() << std::endl;
    }
  }

  return frames;
}
