#ifndef __VSIM_UTIL_FORMAT_HPP__
#define __VSIM_UTIL_FORMAT_HPP__

#include <iostream>
#include <sstream>

#include <vsim/util/detail/format.hpp>

namespace vsim { namespace util {

// Poor man's string formating with non-positional placeholders i.e. format("foo % bar %", 1, 2) -> "foo 1 bar 2";
// Arguments can be any type that defines << operator on a stream

template<typename... Args>
std::string format(const char *s, Args... args) {
    std::ostringstream strm ;
    detail::format(strm, s, args...) ;
    return strm.str() ;
}

// format floating point value
//
// fieldWidth is the minimum number of characters to be printed. If the value to be printed is shorter than this number, the result is padded with blank spaces.
// The value is not truncated even if the result is larger. If this is negative the string is left align in the field space othwerwise it is right aligned.
// format is one of 'f', 'F', 'g', 'G', 'e', 'E' (similar to printf).
// precision is the number of significant digits retained

std::string formatFloat(double arg, int fieldWidth = 0, char format = 'g', int precision = -1, char fill_char = ' ') ;

// format integer value
//
// fieldWidth is the minimum number of characters to be printed. If the value to be printed is shorter than this number, the result is padded with blank spaces.
// The value is not truncated even if the result is larger. If this is negative the string is left align in the field space othwerwise it is right aligned.
// base is one of 'd', 'i', 'u', 'x', 'X', 'o', 'O'.

std::string formatDecimal(long long int arg, int fieldWidth = 0, char base = 'd', char fill_char = ' ') ;


} // namespace util
} // namespace vsim

#endif
