#include <cvx/util/math/rng.hpp>
#include <random>

namespace cvx { namespace util {

RNG::RNG() {
    static std::random_device rd ;
    generator_.seed(rd());
}

RNG::RNG(uint64_t seed): generator_(seed) {

}

}}
