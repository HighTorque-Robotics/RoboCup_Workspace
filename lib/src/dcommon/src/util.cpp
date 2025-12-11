#include "dcommon/util.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace dcommon {
namespace util {

void string_split(const std::string &s, char delim,
                  std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
}

std::vector<std::string> string_split(const std::string &s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
}

void string_replace(std::string &str, const std::string &from,
                    const std::string &to) {
  if (from.empty()) return;
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    // In case 'to' contains 'from', like replacing
    // 'x' with 'yx'
    start_pos += to.length();
  }
}

bool string_contains(const std::string &str, const std::string &other) {
  return str.find(other) != std::string::npos;
}

bool file_exists(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

std::string basename(const std::string &path) {
  std::vector<std::string> elems;
  string_split(path, '/', elems);
  return elems[elems.size() - 1];
}

std::string dirname(const std::string &path) {
  std::vector<std::string> elems;
  string_split(path, '/', elems);

  // if there is no prefix, return "./"
  if (elems.size() < 2) {
    return "./";
  }
  // otherwise concatenate elements
  if (elems.size() < 2) return "./";

  std::ostringstream oss;
  for (size_t idx = 0; idx < elems.size() - 1; idx++) {
    oss << elems[idx] << "/";
  }
  return oss.str();
}

std::string file2string(const std::string &path) {
  std::ifstream in(path, std::ios::in | std::ios::binary);
  if (in) {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return contents;
  }
  throw std::runtime_error("rhoban_utils::file2string: Failed to open file '" +
                           path + "'");
}

std::vector<std::string> file2lines(const std::string &path) {
  std::string content = file2string(path);
  std::vector<std::string> lines;
  string_split(content, '\n', lines);
  return lines;
}

}  // namespace util
}  // namespace dcommon
