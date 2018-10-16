#ifndef __CVX_VIZ_GLYPH_CACHE_HPP__
#define __CVX_VIZ_GLYPH_CACHE_HPP__

#include "font_manager.hpp"
#include "GL/gl3w.h"

#include <hb-ft.h>
#include <array>

namespace cvx { namespace viz { namespace detail {

struct Glyph {
    GLfloat x_ ;
    GLfloat y_ ;
    GLfloat u_ ;
    GLfloat v_ ;
};
struct TextQuads {
    std::vector<Glyph> vertices_ ;
    std::vector<GLuint> indices_ ;
};

class GlyphCache ;
using GlyphCacheMap = std::map< std::pair<FT_Face, size_t>, GlyphCache> ;

class GlyphCache {
public:
    GlyphCache(FT_Face face, size_t pixel_size) ;
    ~GlyphCache() ;

    // performs shaping and composition of quads that may be used for rendering
    void prepare(const std::string &characters, TextQuads &td) ;

    GLuint textureId() const { return texture_ ; }

private:

     using GlyphQuad = std::array<Glyph, 4> ;

    void cache(hb_codepoint_t cp, GlyphQuad &) ;

    FT_Face face_ ;
    size_t sz_ ;
    hb_font_t *font_ ;

    size_t width_ = 256 ;
    size_t height_ = 256 ;
    size_t y_ = 0, x_ = 0, line_height_ = 0 ;

    GLuint texture_ ;

    std::map<hb_codepoint_t, GlyphQuad> glyph_map_ ;

    static const GLuint TEXTURE_UNIT = 0 ;
    static const uint PADDING = 1 ;

public:
    static GlyphCacheMap g_glyphs ;
};



}}}

#endif
