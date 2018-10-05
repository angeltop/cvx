#ifndef __CVX_VIZ_TEXT_HPP__
#define __CVX_VIZ_TEXT_HPP__

#include <memory>
#include <Eigen/Core>

#include <cvx/viz/renderer/font.hpp>

namespace cvx { namespace viz {

namespace detail {
    class TextItem ;
}

// wrapper for text object, usefull for repetitive drawing of the same text
class Text {
public:

    Text(const std::string &t, const Font &f) ;

    void render(float x, float y, const Eigen::Vector3f &clr) ;

private:

    std::shared_ptr<detail::TextItem> impl_ ;
} ;

}}

#endif
