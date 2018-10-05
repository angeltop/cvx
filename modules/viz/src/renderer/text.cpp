#include <cvx/viz/renderer/text.hpp>
#include "text_item.hpp"

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz {

Text::Text(const std::string &t, const Font &f): impl_(new detail::TextItem(t, f)) {
}

void Text::render(float x, float y, const Vector3f &clr) {
    impl_->render(x, y, clr) ;
}



}}
