/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file util.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-06-06
 */

#include <ctime>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dcommon {
namespace util {
/**
 * @brief Split a string by given delim
 * @param s - string to split
 * @param delim - delim character
 * @param elems - splited elements
 */
void string_split(const std::string &s, char delim,
                  std::vector<std::string> &elems);
/**
 * @brief Split a string by given delim
 * @param s - string to split
 * @param delim - delim character
 * @return splited elements
 */
std::vector<std::string> string_split(const std::string &s, char delim);

/**
 * @brief replace given sub-string with desired string
 * @param str - original string
 * @param from - original sub-string
 * @param to - desired sub-string
 */
void string_replace(std::string &str, const std::string &from,
                    const std::string &to);

/**
 * @brief Check string contains given sub-string
 * @param str - original string
 * @param other - sub-string to find
 * @return Whether or not string contains given sub-string
 */
bool string_contains(const std::string &str, const std::string &other);

/**
 * @brief Check whether file exists
 * @param name - file path
 * @return whether or not file exists in given path
 */
bool file_exists(const std::string &name);

/**
 * @brief Get base name of file in given path
 * @param path - file path
 * @return base name of file
 */
std::string basename(const std::string &path);

/**
 * @brief Get directory name of file in given path
 * @param path - file path
 * @return directory name of file
 */
std::string dirname(const std::string &path);

/**
 * @brief Read file content as a string
 * @param path - file path
 * @return file content as a string
 */
std::string file2string(const std::string &path);

/**
 * @brief Read file content as lines
 * @param path - file path
 * @return file content as lines
 */
std::vector<std::string> file2lines(const std::string &path);

}  // namespace util
}  // namespace dcommon
