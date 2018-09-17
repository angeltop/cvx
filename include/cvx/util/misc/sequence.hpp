#ifndef __CVX_UTIL_SEQUENCE_HPP__
#define __CVX_UTIL_SEQUENCE_HPP__

#include <string>

namespace cvx {
namespace util {

// represents a sequence of files that are sequentially numbered <prefix><frame_id><suffix> where <frame_id> is a zero padded fixed length integer

class FileSequence {
public:

    FileSequence(const std::string &prefix, const std::string &suffix, uint ndigits = 5):
        prefix_(prefix), suffix_(suffix), digits_(ndigits) {}

    std::string format(uint fid) ;
    int parseFrameId(const std::string &fileName) ;

private:

    std::string prefix_, suffix_ ;
    uint digits_ ;
} ;

}}

#endif
