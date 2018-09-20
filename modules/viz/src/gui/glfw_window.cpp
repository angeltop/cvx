#include <cvx/viz/gui/glfw_window.hpp>
#include <thread>
#include <iostream>
using namespace std ;

namespace cvx {

glfwRenderWindow::~glfwRenderWindow()
{

}

bool glfwRenderWindow::run(size_t width, size_t height, const string &wname) {

    glfwSetErrorCallback(errorCallback);

    if( !glfwInit() ) return false ;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_FALSE);
    if ( !( handle_ = glfwCreateWindow(width, height, wname.c_str(), 0, 0) )) {
        glfwTerminate();
        return false ;
    }

    glfwSetWindowUserPointer(handle_, this) ;

    glfwMakeContextCurrent(handle_);
  //  glfwSwapInterval(1);
    glfwSetTime( 0.0 );

    glfwSetCursorPosCallback(handle_, moveCallback);
    glfwSetKeyCallback(handle_, keyCallback);
    glfwSetMouseButtonCallback(handle_, buttonCallback);
    glfwSetScrollCallback(handle_, scrollCallback);
    glfwSetWindowSizeCallback(handle_, sizeCallback);

    // this is needed for non core profiles or instead use gl3w
    glewExperimental = GL_TRUE ;

    GLenum err ;
    if ( ( err = glewInit() ) != GLEW_OK ) {
       return false ;
    }

    onInit() ;

    sizeCallback(handle_, width, height); // Set initial size.

    while (!glfwWindowShouldClose(handle_))  {
        double current_time =  glfwGetTime();
        double elapsed_time = current_time - saved_time_;

        if ( elapsed_time >= 1.0/30 ) {
            onRender() ;
            glfwSwapBuffers(handle_);
            saved_time_ = current_time ;
        }

        glfwPollEvents();
    }

    glfwDestroyWindow(handle_);

    glfwTerminate();

    return true ;
}

void glfwRenderWindow::buttonCallback(GLFWwindow *window, int button, int action, int mods) {

    glfwRenderWindow *instance = (glfwRenderWindow *)glfwGetWindowUserPointer(window) ;

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    switch(action)
    {
    case GLFW_PRESS:
        instance->onMouseButtonPressed(button, xpos, ypos, mods) ;
        break ;
    case GLFW_RELEASE:
        instance->onMouseButtonReleased(button, xpos, ypos, mods) ;
        break ;
    default: break;
    }
}

void glfwRenderWindow::errorCallback(int error, const char *description) {

}

void glfwRenderWindow::keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    glfwRenderWindow *instance = (glfwRenderWindow *)glfwGetWindowUserPointer(window) ;

    switch(action) {
    case GLFW_PRESS:
        switch(key)
        {
        case GLFW_KEY_ESCAPE:
            // Exit app on ESC key.
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;
        default:
            instance->onKeyPressed(key, mods);
        }
        break;
    case GLFW_RELEASE:
        instance->onKeyReleased(key, mods) ;
    default: break;
    }

}

void glfwRenderWindow::moveCallback(GLFWwindow *window, double xpos, double ypos) {
     glfwRenderWindow *instance = (glfwRenderWindow *)glfwGetWindowUserPointer(window) ;

     instance->onMouseMoved(xpos, ypos);
}

void glfwRenderWindow::scrollCallback(GLFWwindow *window, double xpos, double ypos) {
    glfwRenderWindow *instance = (glfwRenderWindow *)glfwGetWindowUserPointer(window) ;
    instance->onMouseWheel(xpos + ypos);
}

void glfwRenderWindow::sizeCallback(GLFWwindow *window, int width, int height) {
    glfwRenderWindow *instance = (glfwRenderWindow *)glfwGetWindowUserPointer(window) ;
    instance->onResize(width, height) ;
}

} // namespace cvx
