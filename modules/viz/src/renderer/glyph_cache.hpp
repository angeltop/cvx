#ifndef __CVX_VIZ_GLYPH_CACHE_HPP__
#define __CVX_VIZ_GLYPH_CACHE_HPP__

#include "font_manager.hpp"
#include <hb-ft.h>
#include <GL/glew.h>
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

class GlyphCache {
public:
    GlyphCache(FT_Face face, size_t pixel_size) ;

    // performs shaping and composition of quads that may be used for rendering
    void prepare(const std::string &characters, TextQuads &td) ;

    GLuint textureId() const { return texture_ ; }

private:

    friend class RendererImpl ;



    using GlyphQuad = std::array<Glyph, 4> ;

    void cache(hb_codepoint_t cp, GlyphQuad &) ;

    FT_Face face_ ;
    size_t sz_ ;
    hb_font_t *font_ ;

    size_t width_ = 256 ;
    size_t height_ = 256 ;
    size_t y_ = 0, x_ = 0, line_height_ = 0 ;

    std::array<GLfloat, 8> quad_vertices_ ;
    std::array<GLuint, 6> quad_indices_ ;
    std::array<GLfloat, 8> quad_uvs_ ;
    GLuint vao_, vbo_, ebo_, texture_ ;

    std::map<hb_codepoint_t, GlyphQuad> glyph_map_ ;

    static const GLuint VERTEX_ATTRIBUTE = 0 ;
    static const GLuint UV_ATTRIBUTE = 1 ;
    static const GLuint TEXTURE_UNIT = 0 ;
    static const uint PADDING = 1 ;
};



}}}

#endif
