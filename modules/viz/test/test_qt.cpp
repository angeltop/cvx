#include <QtGui/QtGui>
#include "qt_openglwindow.hpp"

#include <cvx/viz/renderer/renderer.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/node.hpp>


using namespace Eigen ;


using namespace cvx::viz ;

class ExampleWindow : public OpenGLWindow
{
public:
    ExampleWindow(ScenePtr scene);

    void initialize() override;
    void render() override;
    void resize(size_t w, size_t h) override ;


private:

    Renderer rdr_ ;
    CameraPtr camera_ ;

};

void ExampleWindow::initialize()
{
   rdr_.init() ;


}

void ExampleWindow::resize(size_t w, size_t h) {
    float ratio = w/(float)h ;
    std::static_pointer_cast<PerspectiveCamera>(camera_)->setAspectRatio(ratio) ;


    camera_->setViewport(w, h) ;

}


void ExampleWindow::render()
{
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    rdr_.render(camera_) ;
}

ExampleWindow::ExampleWindow(ScenePtr scene): rdr_(scene) {

     auto c = scene->geomCenter() ;
     auto r = scene->geomRadius(c) ;

     // create a camera
     uint width = 640, height = 480 ;
     camera_.reset(new PerspectiveCamera(width / (float) height, // aspect ratio
                                         50*M_PI/180,   // fov
                                         0.0001,        // zmin
                                         10*r           // zmax
                                         ) ) ;
     camera_->lookAt(c + Vector3f{0.0, 0, 2*r}, c, {0, 1, 0}) ;

       camera_->setBgColor({1, 1, 1, 1}) ;

       camera_->setViewport(width, height) ;
}

int main(int argc, char **argv)
{

    ScenePtr scene(new Scene) ;
   //  scene->load("/home/malasiot/Downloads/greek_column.obj") ;
     scene->load("/home/malasiot/Downloads/cube.obj") ;

    DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
    dl->diffuse_color_ = Vector3f(1, 1, 1) ;
    scene->addLight(LightPtr(dl)) ;

    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(16);

    ExampleWindow window(scene) ;
    window.setFormat(format);
    window.resize(640, 480);
    window.show();

    window.setAnimating(true);

    return app.exec();
}
