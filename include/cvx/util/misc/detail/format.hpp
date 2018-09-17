#ifndef __STRINGS_FORMAT_IMPL_HPP__
#define __STRINGS_FORMAT_IMPL_HPP__

#include <string>
#include <memory>
namespace cvx { namespace impl {

/*
   * From Andrei Alexandrescu slides for 'Variadics are funtastic'
   * in Going Native 2012
*/

template <class T> typename std::enable_if<std::is_integral<T>::value, T>::type normalizeArg(T arg) { return arg; }

template <class T> typename std::enable_if<std::is_floating_point<T>::value, double>::type normalizeArg(T arg) { return arg; }

template <class T> typename std::enable_if<std::is_pointer<T>::value, T>::type normalizeArg(T arg) { return arg; }

inline const char* normalizeArg(std::string const& arg) { return arg.c_str(); }

template<typename... Args>
std::string format(const char *frmt, Args... args) {
    // argument checking not implemented
    //   check_printf(frmt, normalizeArg(args)...);
    size_t size = std::snprintf( nullptr, 0, frmt, normalizeArg(args)... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, frmt, normalizeArg(args)... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

}
}

#endif
