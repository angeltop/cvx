#include <cvx/util/misc/sequence.hpp>
#include <cvx/util/misc/strings.hpp>

#include <iomanip>

using namespace std ;

namespace cvx { namespace util {

std::string FileSequence::format(uint fid) {
    stringstream strm ;
    strm << prefix_ << std::setfill('0') << std::setw(digits_) << fid << suffix_ ;
    return strm.str() ;
}


int FileSequence::parseFrameId(const std::string &fileName)
{
    size_t pos = fileName.find_last_of("/\\") ;

    // strip filename if a full path
    string fname = ( pos == string::npos ) ? fileName : fileName.substr(pos+1) ;

    if ( fname.empty() ) return -1 ;

    if ( !startsWith(fname, prefix_) || !endsWith(fname, suffix_) ) return -1 ;

    try {
        int res = stoi(fname.substr(prefix_.size(), digits_)) ;
        return res ;
    }
    catch ( invalid_argument &e ) {
        return -1 ;
    }
}



}}
