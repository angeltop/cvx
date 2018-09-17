#ifndef __STRINGS_HPP__
#define __STRINGS_HPP__

#include <string>
#include <sstream>
#include <vector>
#include <cvx/util/misc/regex.hpp>

#include <cvx/util/misc/detail/format.hpp>

// various string manipulation functions

namespace cvx {

// format floating point value
//
// fieldWidth is the minimum number of characters to be printed. If the value to be printed is shorter than this number, the result is padded with blank spaces.
// The value is not truncated even if the result is larger. If this is negative the string is left align in the field space othwerwise it is right aligned.
// format is one of 'f', 'F', 'g', 'G', 'e', 'E' (similar to printf).
// precision is the number of significant digits retained

std::string format(double arg, int fieldWidth = 0, char format = 'g', int precision = -1, char fill_char = ' ') ;

// format integer value
//
// fieldWidth is the minimum number of characters to be printed. If the value to be printed is shorter than this number, the result is padded with blank spaces.
// The value is not truncated even if the result is larger. If this is negative the string is left align in the field space othwerwise it is right aligned.
// base is one of 'd', 'i', 'u', 'x', 'X', 'o', 'O'.

std::string format(long long int arg, int fieldWidth = 0, char base = 'd', char fill_char = ' ') ;

// Printf like formating of string. snprintf is called internally and there is no type safety. Yet one may pass std::strings as arguments
template<typename... Args>
std::string format(const char *frmt, Args... args ) {
    return impl::format(frmt, args...) ;
}

// tokenize string by spliting with one of delimeters
std::vector<std::string> split(const std::string &s, const char *delimeters) ;

// tokenize string by spliting with regex
std::vector<std::string> split(const std::string &s, const regex &space) ;

// join string list with delimeter
std::string join(const std::vector<std::string> &parts, const char *delimeter) ;

// trimming
std::string rtrimCopy(const std::string &str, const char *delim = " \t\n\r") ;
void rtrim(std::string &str, const char *delim = " \t\n\r") ;
std::string ltrimCopy(const std::string &str, const char *delim = " \t\n\r") ;
void ltrim(std::string &str, const char *delim = " \t\n\r") ;
std::string trimCopy(const std::string &str, const char *delim = " \t\n\r") ;
void trim(std::string &str, const char *delim = " \t\n\r") ;

// prefix
bool startsWith(const std::string &src, const std::string &prefix) ;
// suffix
bool endsWith(const std::string &src, const std::string &suffix) ;

// search & replace

void replaceAll(std::string &src, const std::string &subject, const std::string &replacement) ;
std::string replaceAllCopy(const std::string &src, const std::string &subject, const std::string &replacement) ;

// convertion
std::string toUpperCopy(const std::string &source) ;
void toUpper(std::string &source) ;
std::string toLowerCopy(const std::string &source) ;
void toLower(std::string &source) ;

// regex replace, uses std::regex_replace
std::string replace(std::string &src, const regex &subject, const std::string &replacement) ;
// regex replace with a callback function
std::string replace(std::string &src, const regex &subject, std::function<std::string (const smatch &)> callback) ;

}

#endif
