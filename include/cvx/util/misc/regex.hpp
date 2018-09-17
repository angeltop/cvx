#ifndef __CVX_UTIL_REGEX_HPP__
#define __CVX_UTIL_REGEX_HPP__

// hack to deal with broken regex in gcc < 4.9

#ifdef USE_BOOST_REGEX
#include <boost/regex.hpp>
using boost::regex;
using boost::smatch;
using boost::match_results;
using boost::regex_error;
using boost::sregex_token_iterator ;
using boost::sregex_iterator ;
#else
#include <regex>
using std::regex;
using std::smatch;
using std::match_results;
using std::regex_error;
using std::sregex_token_iterator ;
using std::sregex_iterator ;
#endif


#endif
