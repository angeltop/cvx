#include <cvx/viz/renderer/renderer.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/scene/mesh.hpp>
#include <cvx/viz/scene/material.hpp>

#include <cvx/viz/gui/glfw_window.hpp>
#include <cvx/viz/gui/trackball.hpp>

#include <cvx/util/imgproc/rgbd.hpp>
#include <cvx/util/camera/camera.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cvx::viz ;
using namespace cvx::util ;


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

    void onMouseWheel(double x) {
        trackball_.setScrollDirection(x>0);
    }

    void onMouseMoved(double xpos, double ypos) override {
        trackball_.setClickPoint(xpos, ypos) ;
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

    cv::Mat depth = cv::imread("/home/malasiot/Downloads/grab_rgbd/soap/depth_00001.png", -1) ;
    cv::Mat clr = cv::imread("/home/malasiot/Downloads/grab_rgbd/soap/rgb_00001.png") ;

    PinholeCamera cam(525, 525, 640/2.0, 480/2, cv::Size(640, 480)) ;

    PointList3f coords, clrs ;
    depthToPointCloud(depth, cam, coords) ;

    uint w = clr.cols, h = clr.rows ;

    for(uint i=0 ; i<h ; i++ )
        for(uint j=0 ; j<w ; j++ ) {
            if ( depth.at<ushort>(i, j) != 0 ) {
                const cv::Vec3b &c = clr.at<cv::Vec3b>(i, j) ;
                clrs.emplace_back(c[2]/255.0, c[1]/255.0, c[0]/255.0) ;
            }
        }


    scene->addSimpleShapeNode(Mesh::makePointCloud(coords, clrs), MaterialInstancePtr(new PerVertexColorMaterialInstance(1))) ;

  /*
    DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
    dl->diffuse_color_ = Vector3f(1, 1, 1) ;
    scene->addLight(LightPtr(dl)) ;
*/
    glfwGUI gui(scene) ;

    gui.run(640, 480) ;

}
