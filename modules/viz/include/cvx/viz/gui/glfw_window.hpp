#ifndef __GLFW_RENDER_WINDOW_HPP__
#define __GLFW_RENDER_WINDOW_HPP__

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <string>

namespace cvx {


class glfwRenderWindow
{
public:
    glfwRenderWindow() = default ;
    ~glfwRenderWindow();

    bool run(size_t width, size_t height, const std::string &name = "window");

    virtual void onInit() {} // one time initialization of renderer
    virtual void onRender() {} // called to render every frame

    virtual void onMouseButtonPressed(uint button, size_t x, size_t y, uint flags) {}
    virtual void onMouseButtonReleased(uint button, size_t x, size_t y, uint flags) {}

    virtual void onKeyPressed(size_t key_code, uint mods) {}
    virtual void onKeyReleased(size_t key_code, uint mods) {}

    virtual void onMouseMoved(double xpos, double ypos) {}
    virtual void onMouseWheel(double x) {}
    virtual void onResize(int width, int height) {}

    static glfwRenderWindow *instance() {
        static glfwRenderWindow *instance_ ;
        return instance_ ;
    }

private:

    static void buttonCallback(GLFWwindow *window, int button, int action, int mods);
    static void errorCallback(int error, const char* description);
    static void keyCallback(GLFWwindow *window, int key, int scancode,
                                int action, int mods);
    static void moveCallback(GLFWwindow *window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow *window, double xpos, double ypos);
    static void sizeCallback(GLFWwindow *window, int width, int height);

protected:

    GLFWwindow *handle_ ;
    double saved_time_ = 0 ;

};

} // namespace cvx

#endif
