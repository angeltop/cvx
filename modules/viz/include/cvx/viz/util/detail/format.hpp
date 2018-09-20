#ifndef __VSIM_UTIL_FORMAT_DETAIL_HPP__
#define __VSIM_UTIL_FORMAT_DETAIL_HPP__

#include <iostream>
#include <sstream>

namespace vsim { namespace util {

namespace detail {

inline void format(std::ostream &strm, const char *s) {
    while (*s) {
        if (*s == '%') {
            if (*(s + 1) == '%') ++s;
            else throw "invalid format string: missing arguments";
        }
        strm << *s++;
    }
}

template<typename T, typename... Args>
void format(std::ostream &strm, const char *s, T& value, Args... args)
{
    while (*s) {
        if (*s == '%') {
            if (*(s + 1) == '%') ++s;
            else {
                strm << value;
                format(strm, s + 1, args...); // call even when *s == 0 to detect extra arguments
                return;
            }
        }
        strm << *s++;
    }
    throw "extra arguments provided to format";
}

} // namespace detail
} // namespace util
} // namespace vsim

#endif
