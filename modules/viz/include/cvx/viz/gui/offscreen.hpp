#ifndef __OFFSCREEN_RENDER_WINDOW_HPP__
#define __OFFSCREEN_RENDER_WINDOW_HPP__

#include <stdexcept>
#include <memory>

namespace cvx { namespace viz {

namespace impl {
    class OffscreenRenderingWindow ;
}

class OffscreenRenderingWindow {
public:

    OffscreenRenderingWindow(uint32_t w, uint32_t h);

private:
    std::shared_ptr<impl::OffscreenRenderingWindow> impl_ ;
};

} // namespace viz
} // namespace cvx

#endif
