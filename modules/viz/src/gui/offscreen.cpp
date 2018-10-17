#include <cvx/viz/gui/offscreen.hpp>

#include "../renderer/GL/gl3w.h"

#include <GLFW/glfw3.h>

namespace cvx { namespace viz {

namespace impl {

class OffscreenRenderingWindow {
public:

    OffscreenRenderingWindow(uint32_t w, uint32_t h);

    bool init();

    ~OffscreenRenderingWindow();

    bool create_buffers();

    GLFWwindow *handle_ ;
    GLuint fbo_, texture_id_, rbo_ ;
    uint32_t width_, height_ ;

};

OffscreenRenderingWindow::OffscreenRenderingWindow(uint32_t w, uint32_t h): width_(w), height_(h) {
    if ( !init()  ) throw std::runtime_error("failed to initialize GL context") ;
}

bool OffscreenRenderingWindow::init() {
    if( !glfwInit() ) return false ;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

    handle_ = glfwCreateWindow(width_, height_, "window", 0, 0);

    if ( !handle_ )     {
        glfwTerminate();
        return false ;
    }

    glfwMakeContextCurrent(handle_) ;

    if ( gl3wInit() != 0 )
        return false ;

    create_buffers() ;

    return true ;
}

OffscreenRenderingWindow::~OffscreenRenderingWindow() {
    if ( texture_id_ )
        glDeleteTextures(1, &texture_id_ );

    texture_id_ = 0;

    // clean up FBO
    if ( fbo_ ) glDeleteFramebuffers(1, &fbo_);
    if ( rbo_ ) glDeleteRenderbuffers(1, &rbo_) ;

    glfwDestroyWindow(handle_);
    glfwTerminate();
}

bool OffscreenRenderingWindow::create_buffers() {
    // create a framebuffer object
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    // create a texture object
    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_id_, 0);

    // create a renderbuffer object to store depth info
    glGenRenderbuffers(1, &rbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo_);

    // bind buffers
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_) ;

    return (fbo_ != 0) ;
}
}

OffscreenRenderingWindow::OffscreenRenderingWindow(uint32_t w, uint32_t h): impl_(new impl::OffscreenRenderingWindow(w, h) ) {

}


}}
