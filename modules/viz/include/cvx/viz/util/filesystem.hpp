#ifndef _VSIM_UTIL_FILSYSTEM_HPP__
#define _VSIM_UTIL_FILSYSTEM_HPP__

#include <string>

namespace vsim { namespace util {


void split_path(const std::string &path, std::string &dir, std::string &basename, std::string &extension) ;

std::string basename(const std::string &path) ;

std::string get_file_contents(const std::string &fname) ;



} // namespace util
} // namespace vsim
#endif
