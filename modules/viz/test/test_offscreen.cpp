#include <cvx/viz/renderer/renderer.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/node.hpp>

#include <cvx/viz/gui/offscreen.hpp>

#include <iostream>

using namespace cvx::viz ;

using namespace std ;
using namespace Eigen ;


int main(int argc, char *argv[]) {

    // load scene

    ScenePtr scene(new Scene) ;
   // scene->load("/home/malasiot/Downloads/greek_column.obj") ;
    scene->load("/home/malasiot/Downloads/cube.obj") ;

    // optional compute center and radius to properly position camera
    auto c = scene->geomCenter() ;
    auto r = scene->geomRadius(c) ;

    // add a ligh source

    DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
    dl->diffuse_color_ = Vector3f(1, 1, 1) ;
    scene->addLight(LightPtr(dl)) ;

    // create a camera
    uint width = 240, height = 480 ;
    PerspectiveCamera *pcam = new PerspectiveCamera(width / (float) height, // aspect ratio
                                                    50*M_PI/180,   // fov
                                                    0.0001,        // zmin
                                                    10*r           // zmax
                                                    ) ;

    CameraPtr cam(pcam) ;

    cam->setBgColor({0, 0, 0, 1});

    // position camera to look at the center of the object

    pcam->lookAt(c + Vector3f{0.0, 0, 2*r}, c, {0, 1, 0}) ;

    // set camera viewpot

    pcam->setViewport(width, height)  ;

    // create the offscreen window

    OffscreenRenderingWindow ow(width, height) ;

    // initialize renderer

    Renderer rdr(scene) ;
    rdr.init() ;


    // render scene (possibly multiple times)
    rdr.render(cam) ;
    rdr.text("ABCDabcd", 100, 100, Font("arial", 32), { 1, 0, 0} ) ;
    // obtain the color buffer
    cv::Mat clr = rdr.getColor(true) ;

    cv::imwrite("/tmp/oo.png", clr) ;

}

