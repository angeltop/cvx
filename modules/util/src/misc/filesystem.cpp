#include <cvx/util/misc/filesystem.hpp>

#include <algorithm>
#include <fstream>

using namespace std ;

namespace cvx { namespace util {

string get_file_contents(const std::string &fname) {
    ifstream strm(fname) ;

    if ( !strm.good() ) return string() ;
    strm.seekg(0, ios::end);
    size_t length = strm.tellg();
    strm.seekg(0,std::ios::beg);

    string res ;
    res.resize(length) ;
    strm.read(&res[0], length) ;

    return res ;
}


}}

