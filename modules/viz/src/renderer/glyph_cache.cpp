#include "glyph_cache.hpp"
#include <hb-ft.h>
#include <cmath>
#include <iostream>

using namespace std ;

namespace cvx { namespace viz { namespace detail {

GlyphCache::GlyphCache(FT_Face face, size_t px): face_(face), sz_(px) {

      FT_Set_Pixel_Sizes(face, 0, px);

      font_ = hb_ft_font_create_referenced(face);

      // Calculate actual font size
      size_t maxSlotWidth = static_cast<size_t>(
          ceil((face->max_advance_width * face->size->metrics.y_ppem) / static_cast<float>(face->units_per_EM)));
      size_t maxSlotHeight = static_cast<size_t>(
           ceil((face->height * face->size->metrics.y_ppem) / static_cast<float>(face->units_per_EM)));

     cout <<"ok" ;
}

void GlyphCache::cache(const std::string &characters)
{

}

}}}
