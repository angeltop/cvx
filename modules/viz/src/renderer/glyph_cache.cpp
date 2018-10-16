#include "glyph_cache.hpp"
#include <hb-ft.h>
#include <cmath>
#include <iostream>
#include <memory>

using namespace std ;

// adopted from https://github.com/SRombauts/gltext/

namespace cvx { namespace viz { namespace detail {

GlyphCacheMap GlyphCache::g_glyphs ;

GlyphCache::GlyphCache(FT_Face face, size_t px): face_(face), sz_(px) {

    FT_Set_Pixel_Sizes(face_, 0, sz_);
    font_ = hb_ft_font_create(face_, 0);

    // create texture atlas

    glActiveTexture(GL_TEXTURE0 + TEXTURE_UNIT);
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    std::vector<GLubyte> empty_data(width_* height_ * sizeof(GLfloat), 0); // transparent black texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width_, height_, 0, GL_RED, GL_UNSIGNED_BYTE, &empty_data[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

}

GlyphCache::~GlyphCache() {
    hb_font_destroy(font_);
    glDeleteTextures(1, &texture_);
}

void GlyphCache::prepare(const std::string &text, TextQuads &data)
{
    // Put the provided UTF-8 encoded characters into a Harfbuzz buffer

    std::unique_ptr<hb_buffer_t, void (*)(hb_buffer_t *)> buffer(hb_buffer_create(), &hb_buffer_destroy);

    hb_buffer_set_direction(buffer.get(), HB_DIRECTION_LTR);
    hb_buffer_add_utf8(buffer.get(), text.c_str(), text.size(), 0, text.size());
    // Ask Harfbuzz to shape the UTF-8 buffer
    hb_shape(font_, buffer.get(), nullptr, 0);

    // Get buffer properties
    size_t text_length = hb_buffer_get_length(buffer.get());
    hb_glyph_info_t* glyphs = hb_buffer_get_glyph_infos(buffer.get(), nullptr);
    hb_glyph_position_t* positions = hb_buffer_get_glyph_positions(buffer.get(), nullptr);

    glActiveTexture(GL_TEXTURE0 + TEXTURE_UNIT);
    glBindTexture(GL_TEXTURE_2D, texture_);

    vector<GlyphQuad> quads ;
    // Iterate over the glyphs of the text and cache characters (codepoints) not already cached
    // fill in quads
    for ( size_t i = 0; i < text_length ; ++i ) {
        const auto it = glyph_map_.find(glyphs[i].codepoint) ;
        if ( it == glyph_map_.end() ) {
            GlyphQuad quad ;
            cache(glyphs[i].codepoint, quad) ;
            glyph_map_.emplace(glyphs[i].codepoint, quad) ;
            quads.emplace_back(quad) ;
        }
        else quads.emplace_back(it->second) ;
    }

    size_t position_x = 0;
    size_t position_y = 0;

    // Iterate over the glyphs of the text

    for ( size_t i = 0; i < text_length; ++i ) {

        GlyphQuad &quad = quads[i] ;

        // offset each quad according to hurfbazz shaping

        for( uint k=0 ; k<4 ; k++ ) {
            Glyph &q = quad[k] ;
            data.vertices_.emplace_back( Glyph{ q.x_ + position_x + positions[i].x_offset, q.y_ + position_y + positions[i].y_offset , q.u_, q.v_} ) ;
        }

        GLuint ioffset = 4 * i ;

        data.indices_.insert(data.indices_.end(), { ioffset, ioffset + 1, ioffset + 2, ioffset + 1, ioffset + 2, ioffset + 3}) ;

        position_x += (positions[i].x_advance >> 6);
    }

}


void GlyphCache::cache(hb_codepoint_t cp, GlyphQuad &quad) {
    const FT_Bitmap& bitmap = face_->glyph->bitmap;

    if ( FT_Load_Glyph(face_, cp, FT_LOAD_RENDER) != 0 ) {
        throw std::runtime_error("FT_Load_Glyph") ;
    }

    if ( x_ + bitmap.width >= width_  ) { // need to start a new line
        y_ += line_height_ + PADDING ;
        x_ = line_height_ = 0 ;
    }

    if ( y_ + bitmap.rows >= height_ ) { // if there is no more space panic
        throw std::runtime_error("overflow") ;
    }

    // Recalculate height of the current row based on height of the new Glyph

    line_height_ = std::max<size_t>(line_height_, bitmap.rows) ;

    // The pitch is positive when the bitmap has a `down' flow, and negative when it has an `up' flow.
    // In all cases, the pitch is an offset to add to a bitmap pointer in order to go down one row.

    int pitch = bitmap.pitch;
    if (pitch < 0) {
        pitch = -pitch;
    }

    // GL_UNPACK_ROW_LENGTH defines the number of pixels in a row
  //  glPixelStorei(GL_UNPACK_ROW_LENGTH, pitch);

    // paste bitmap rendered by FT to texture memory

    glTexSubImage2D(GL_TEXTURE_2D, 0,  x_, y_, bitmap.width, bitmap.rows, GL_RED, GL_UNSIGNED_BYTE, bitmap.buffer);

    // compute glyph position on the atlas

    // ^ y/t
    // |
    // 2 - 3
    // | \ |
    // 0 - 1 -> x/s

    float offset_x = face_->glyph->bitmap_left;
    float offset_y = face_->glyph->bitmap_top - bitmap.rows; // Can be negative

    quad = {
        Glyph{ offset_x, offset_y, x_/static_cast<float>(width_), (y_ + bitmap.rows)/static_cast<float>(height_) },
        Glyph{ offset_x + bitmap.width, offset_y, (x_ + bitmap.width)/static_cast<float>(width_), (y_ + bitmap.rows)/static_cast<float>(height_) },
        Glyph{ offset_x, offset_y + bitmap.rows, x_/static_cast<float>(width_), y_/static_cast<float>(height_) },
        Glyph{ offset_x + bitmap.width, offset_y + bitmap.rows, (x_ + bitmap.width)/static_cast<float>(width_), y_/static_cast<float>(height_)}
    } ;

   //  advance cursor to hold the new glyph
   x_ += bitmap.width + PADDING ;

   if ( x_ >= width_ ) {
       x_ = 0 ;
       y_ += line_height_ ;
       line_height_ = 0 ;
    }


}


}}}
