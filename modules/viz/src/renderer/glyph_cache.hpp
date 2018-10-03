#ifndef __CVX_VIZ_GLYPH_CACHE_HPP__
#define __CVX_VIZ_GLYPH_CACHE_HPP__

#include "font_manager.hpp"
#include <hb-ft.h>

namespace cvx { namespace viz { namespace detail {

class GlyphCache {
public:
    GlyphCache(FT_Face face, size_t pixel_size) ;

    void cache(const std::string &characters) ;

private:

    FT_Face face_ ;
    size_t sz_ ;
    hb_font_t *font_ ;

    size_t width_ = 256 ;
    size_t height_ = 256 ;
    size_t row_ = 0, col_ = 0 ;
    uint texture_id_ ;
};



}}}

#endif
