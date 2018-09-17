#ifndef __RANDOM_HPP__
#define __RANDOM_HPP__

#include <vector>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

// Some random number helpers

namespace cvx { namespace util {

template<class R>
struct std_rng : std::unary_function<unsigned, unsigned> {
    R &state_;
    unsigned operator()(unsigned i) {
        boost::uniform_int<> rng(0, i - 1);
        return rng(state_);
    }
    std_rng(R &state) : state_(state) {}
};

// sample n random points from a dataset of size N

template<class sample_idx_t>
void sample_with_replacement(uint32_t n, sample_idx_t N, std::vector<sample_idx_t> &subset) {

    namespace rnd = boost::random ;
        // Random generator
    rnd::random_device rdev ;
    rnd::mt19937 rng(rdev);

    sample_idx_t max_idx = static_cast<sample_idx_t>(N-1) ;

    std::vector<sample_idx_t> vidx ;
    for( int i=0 ; i<N ; i++ ) vidx.push_back(i) ;

    for ( int i=0; i<n ; i++ )
    {
        rnd::uniform_int_distribution <> ud(0, max_idx) ;
        int index = ud(rng) ;
        std::swap(vidx[index],vidx[max_idx]);
        subset.push_back(vidx[max_idx]);
        max_idx -- ;
    }

}


}
}



#endif
