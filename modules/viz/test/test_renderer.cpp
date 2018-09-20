#include <cvx/viz/renderer/renderer.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/node.hpp>

#include <cvx/viz/gui/glfw_window.hpp>
#include <cvx/viz/gui/trackball.hpp>

#include <iostream>

using namespace cvx::viz ;

using namespace std ;
using namespace Eigen ;

class glfwGUI: public glfwRenderWindow {
public:

    glfwGUI(ScenePtr scene): glfwRenderWindow(), rdr_(scene) {
        auto c = scene->geomCenter() ;
        auto r = scene->geomRadius(c) ;

        camera_.reset(new PerspectiveCamera(1.0, 50*M_PI/180, 0.0001, 10*r)) ;
        trackball_.setCamera(camera_, c + Vector3f{0.0, 0, 2*r}, c, {0, 1, 0}) ;
        trackball_.setZoomScale(0.1*r) ;
    }

    void onInit() {
        rdr_.init() ;

    }

    void onResize(int width, int height) {
        float ratio;
        ratio = width / (float) height;

        trackball_.setScreenSize(width, height);

        static_pointer_cast<PerspectiveCamera>(camera_)->setAspectRatio(ratio) ;

        camera_->setViewport(width, height)  ;
    }


    void onMouseButtonPressed(uint button, size_t x, size_t y, uint flags) override {
        switch ( button ) {
            case GLFW_MOUSE_BUTTON_LEFT:
                trackball_.setLeftClicked(true) ;
                break ;
            case GLFW_MOUSE_BUTTON_MIDDLE:
                trackball_.setMiddleClicked(true) ;
                break ;
            case GLFW_MOUSE_BUTTON_RIGHT:
                trackball_.setRightClicked(true) ;
                break ;
        }
        trackball_.setClickPoint(x, y) ;
    }

    void onMouseButtonReleased(uint button, size_t x, size_t y, uint flags) override {
        switch ( button ) {
            case GLFW_MOUSE_BUTTON_LEFT:
                trackball_.setLeftClicked(false) ;
                break ;
            case GLFW_MOUSE_BUTTON_MIDDLE:
                trackball_.setMiddleClicked(false) ;
                break ;
            case GLFW_MOUSE_BUTTON_RIGHT:
                trackball_.setRightClicked(false) ;
                break ;
        }

    }

    void onMouseMoved(double xpos, double ypos) override {
        ostringstream s ;
        s << xpos << ',' << ypos ;
        text_ = s.str() ;

        trackball_.setClickPoint(xpos, ypos) ;
    }

    void onMouseWheel(double x) {
        trackball_.setScrollDirection(x>0);
    }


    void onRender() {
        trackball_.update() ;
        rdr_.render(camera_) ;
    }

    string text_ ;
    Renderer rdr_ ;
    ScenePtr scene_ ;
    TrackBall trackball_ ;
    CameraPtr camera_ ;
};

int main(int argc, char *argv[]) {


    ScenePtr scene(new Scene) ;
    // scene->load("/home/malasiot/Downloads/greek_column.obj") ;
     scene->load("/home/malasiot/Downloads/cube.obj") ;


    DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
    dl->diffuse_color_ = Vector3f(1, 1, 1) ;
    scene->addLight(LightPtr(dl)) ;

    glfwGUI gui(scene) ;

    gui.run(640, 480) ;

}
