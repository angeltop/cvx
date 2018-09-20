#ifndef __CVX_UTIL_OPTIONAL_HPP__
#define __CVX_UTIL_OPTIONAL_HPP__

/* just wrapping 3rdparty implementation of optional to support c++11 */
/* it fallbacks to std::optional for c++17 and above */

#include <cvx/util/misc/detail/optional.hpp>

namespace cvx {
    using nonstd::optional ;
}

#endif
