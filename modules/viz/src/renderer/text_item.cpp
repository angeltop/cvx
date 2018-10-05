#include "text_item.hpp"
#include "glyph_cache.hpp"
#include <cvx/viz/renderer/ogl_shaders.hpp>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz { namespace detail {

TextItem::TextItem(const string &text, const Font &font)
{
    FT_Face f = FontManager::instance().queryFontFace(font) ;

    if ( f == nullptr ) return ;

    auto it = GlyphCache::g_glyphs.find(make_pair(f, font.size())) ;
    if ( it ==  GlyphCache::g_glyphs.end() )
        it =  GlyphCache::g_glyphs.emplace(std::piecewise_construct,
                             std::forward_as_tuple(f, (size_t)font.size()),
                             std::forward_as_tuple(f, font.size())).first ;

    GlyphCache &cache = it->second ;

    cache.prepare(text, quads_) ;
    texture_ = cache.textureId() ;

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);

    glBufferData(GL_ARRAY_BUFFER,   quads_.vertices_.size() * sizeof(Glyph), &quads_.vertices_[0], GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, quads_.indices_.size() * sizeof(GLuint),  &quads_.indices_[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(VERTEX_ATTRIBUTE);
    glVertexAttribPointer(VERTEX_ATTRIBUTE, 2, GL_FLOAT, GL_FALSE, sizeof(Glyph), nullptr);
    glEnableVertexAttribArray(UV_ATTRIBUTE);
    glVertexAttribPointer(UV_ATTRIBUTE, 2, GL_FLOAT, GL_FALSE, sizeof(Glyph), reinterpret_cast<GLvoid*>(sizeof(Glyph)/2));
}


/// Source of the vertex shader used to scale the glyphs vertices
static const char* text_vertex_shader_code = R"(
#version 330

layout(location = 0) in vec2 position;
layout(location = 1) in vec2 texCoord;

smooth out vec2 smoothTexCoord;
uniform mat4 projection;

uniform float aspect ;
uniform vec2 scale;
uniform vec2 offset;

void main() {
    gl_Position =  vec4((position + offset)*scale - vec2(1, 1), 0.0f, 1.0f);
    smoothTexCoord = texCoord;
}
)";

/// Source of the fragment shader used to draw the glyphs using the cache texture
static const char* text_fragment_shader_code = R"(
#version 330

smooth in vec2 smoothTexCoord;
out vec4 outputColor;

uniform sampler2D textureCache;
uniform vec3 color;

void main() {
    float textureIntensity = texture(textureCache, smoothTexCoord).r;
    outputColor = vec4(color, textureIntensity);
}
)";

static OpenGLShaderProgram::Ptr get_text_program() {

    static OpenGLShaderProgram::Ptr prog_ ;

    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, text_vertex_shader_code, "text_vertex_shader")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, text_fragment_shader_code, "text_fragment_shader"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}


void TextItem::render(float x, float y, const Eigen::Vector3f &clr)
{
    glEnable(GL_BLEND);
    // glBlendFunc (GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);

    glDisable(GL_DEPTH_TEST) ;
    glDisable(GL_CULL_FACE) ;

    OpenGLShaderProgram::Ptr prog = get_text_program() ;

    prog->use() ;

    GLint viewport[4];
    glGetIntegerv( GL_VIEWPORT, viewport );
    GLint w = viewport[2], h = viewport[3] ;

    float aspect = w/static_cast<float>(h);

    prog->setUniform("offset", Vector2f(x, y)) ;
    prog->setUniform("aspect", aspect) ;
    prog->setUniform("scale", Vector2f(2.0f/w, 2.0f/h)) ;
    prog->setUniform("color", clr) ;

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    /*

       std::vector<GLubyte> atlas_img(256*265*sizeof(GLubyte));
      glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_BYTE, &atlas_img[0]);

       cv::Mat im(256, 256, CV_8U, &atlas_img[0]) ;
       cv::imwrite("/tmp/atlas.png", im) ;
*/
    // Bind to sampler name zero == the currently bound texture's sampler state becomes active (no dedicated sampler)
    glBindSampler(0, 0);

    // Draw the rendered text
    glBindVertexArray(vao_);

#if 0

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, tfb);

    glEnable(GL_RASTERIZER_DISCARD);

    glBeginTransformFeedback(GL_TRIANGLES);

    glDrawElements(GL_TRIANGLES, quads.indices_.size(), GL_UNSIGNED_INT, 0);

    glEndTransformFeedback();

    glDisable(GL_RASTERIZER_DISCARD);

    vector<GLfloat> fdata(quads.vertices_.size() * 4, 0) ;

    glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, quads.vertices_.size() * 4 * sizeof(GLfloat), fdata.data());

    glBindVertexArray(0) ;
#else
    glDrawElements(GL_TRIANGLES, quads_.indices_.size(), GL_UNSIGNED_INT, 0);
#endif


    glBindVertexArray(0) ;
}

TextItem::~TextItem() {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
}

}}}
