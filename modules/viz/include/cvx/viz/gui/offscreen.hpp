#ifndef __OFFSCREEN_RENDER_WINDOW_HPP__
#define __OFFSCREEN_RENDER_WINDOW_HPP__

#include <stdexcept>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <string>

namespace cvx { namespace viz {

class OffscreenRenderingWindow {
public:

    OffscreenRenderingWindow(uint32_t w, uint32_t h): width_(w), height_(h) {
        if ( !init()  ) throw std::runtime_error("failed to initialize GL context") ;
    }

    bool init() {
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

        // this is needed for non core profiles or instead use gl3w
        glewExperimental = GL_TRUE ;

        GLenum err ;
        if ( ( err = glewInit() ) != GLEW_OK ) {
           return false ;
        }

        create_buffers() ;

        return true ;
    }

    ~OffscreenRenderingWindow() {
        if ( texture_id_ )
            glDeleteTextures(1, &texture_id_ );

        texture_id_ = 0;

        // clean up FBO
        if ( fbo_ ) glDeleteFramebuffers(1, &fbo_);
        if ( rbo_ ) glDeleteRenderbuffers(1, &rbo_) ;

        glfwDestroyWindow(handle_);
        glfwTerminate();
    }

    bool create_buffers() {
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

    GLFWwindow *handle_ ;
    GLuint fbo_, texture_id_, rbo_ ;
    uint32_t width_, height_ ;

};

} // namespace viz
} // namespace cvx

#endif
