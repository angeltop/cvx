#include <GL/glew.h>

#include <QtGui/QtGui>
#include <QApplication>
#include <QMainWindow>
#include "qt_glwidget.hpp"

#include <cvx/util/misc/strings.hpp>
#include <cvx/util/math/rng.hpp>
#include <cvx/viz/renderer/renderer.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/material.hpp>
#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/scene/geometry.hpp>
#include <cvx/viz/gui/trackball.hpp>

#include <iostream>

using namespace Eigen ;

using namespace std ;
using namespace cvx::viz ;
using namespace cvx::util ;


RNG g_rng ;

Matrix3f makeSkewSymmetric(const Vector3f& v) {
    Matrix3f result = Matrix3f::Zero();

    result(0, 1) = -v(2);
    result(1, 0) =  v(2);
    result(0, 2) =  v(1);
    result(2, 0) = -v(1);
    result(1, 2) = -v(0);
    result(2, 1) =  v(0);

    return result;
}

#define EPSILON_EXPMAP_THETA 1.0e-3

Eigen::Matrix3f expMapRot(const Vector3f& q) {
    float theta = q.norm();

    Matrix3f R = Matrix3f::Zero();
    Matrix3f qss =  makeSkewSymmetric(q);
    Matrix3f qss2 =  qss*qss;

    if (theta < EPSILON_EXPMAP_THETA)
        R = Matrix3f::Identity() + qss + 0.5*qss2;
    else
        R = Eigen::Matrix3f::Identity()
                + (sin(theta)/theta)*qss
                + ((1-cos(theta))/(theta*theta))*qss2;

    return R;
}

Isometry3f getRandTransform(double d)
{
    Isometry3f t = Isometry3f::Identity();

    Vector3f rotation(g_rng.uniform(-M_PI, M_PI),  g_rng.uniform(-M_PI, M_PI), g_rng.uniform(-M_PI, M_PI)) ;
    Vector3f position(g_rng.uniform(-0.8, 0.8), g_rng.uniform(d, d + 0.1), g_rng.uniform(-0.8, 0.8));

    t.translation() = position;
    t.linear() = expMapRot(rotation);

    return t;
}

NodePtr randomBox(const string &name, MaterialPtr material, const Vector3f &hs) {

    NodePtr box_node(new Node) ;
    box_node->setName(name) ;

    GeometryPtr geom(new BoxGeometry(hs)) ;

    DrawablePtr dr(new Drawable(geom, material)) ;

    box_node->addDrawable(dr) ;

    box_node->matrix() = getRandTransform(0) ;

    return box_node ;
}

class ExampleWindow : public GLWidget
{
public:
    ExampleWindow(ScenePtr scene);

    void hit(int mouse_x, int mouse_y) {

        Ray ray = camera_->getRay(mouse_x, mouse_y);

       Hit h ;
        if ( scene_->hit(ray, h) ) {

            cout << h.node_->name() << ' ' << h.t_ << "  " ;

            cout << endl ;
        }
    }

private:


    void mousePressEvent(QMouseEvent *event) override
    {
        switch ( event->button() ) {
        case Qt::LeftButton:
            trackball_.setLeftClicked(true) ;
            break ;
        case Qt::MiddleButton:
            trackball_.setMiddleClicked(true) ;
            break ;
        case Qt::RightButton:
            trackball_.setRightClicked(true) ;
            break ;
        }


        hit(event->x(), event->y()) ;

    }

    void mouseReleaseEvent(QMouseEvent * event)
    {
        switch ( event->button() ) {
        case Qt::LeftButton:
            trackball_.setLeftClicked(false) ;
            break ;
        case Qt::MiddleButton:
            trackball_.setMiddleClicked(false) ;
            break ;
        case Qt::RightButton:
            trackball_.setRightClicked(false) ;
            break ;
        }

        trackball_.update() ;

    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        int x = event->x() ;
        int y = event->y() ;

        trackball_.setClickPoint(x, y) ;
        trackball_.update() ;
        update() ;

    }

    void wheelEvent ( QWheelEvent * event ) override {
        trackball_.setScrollDirection(event->delta()>0);
        trackball_.update() ;
        update() ;
    }

    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

    Renderer rdr_ ;
    CameraPtr camera_ ;
    TrackBall trackball_ ;
    ScenePtr scene_ ;

};

void ExampleWindow::initializeGL()
{
    glewExperimental = GL_TRUE ;

    glewInit() ;

   rdr_.init() ;


}

void ExampleWindow::resizeGL(int w, int h) {
    float ratio = w/(float)h ;
    std::static_pointer_cast<PerspectiveCamera>(camera_)->setAspectRatio(ratio) ;

    trackball_.setScreenSize(w, h);
    camera_->setViewport(w, h) ;

}


void ExampleWindow::paintGL()
{

    rdr_.render(camera_) ;
}

ExampleWindow::ExampleWindow(ScenePtr scene): scene_(scene), rdr_(scene) {



     //auto c = scene->geomCenter() ;
    // auto r = scene->geomRadius(c) ;


     Vector3f c {0, 0, 0} ;
     float r = 1.0 ;

     // create a camera
     uint width = 640, height = 480 ;

     camera_.reset(new PerspectiveCamera(1.0, 50*M_PI/180, 0.0001, 10*r)) ;
     trackball_.setCamera(camera_, c + Vector3f{0.0, 0, 2*r}, c, {0, 1, 0}) ;
     trackball_.setZoomScale(0.1*r) ;

     camera_->setBgColor({1, 1, 1, 1}) ;


}

int main(int argc, char **argv)
{

    ScenePtr scene(new Scene) ;

   //  scene->load("/home/malasiot/Downloads/greek_column.obj") ;
    // scene->load("/home/malasiot/Downloads/cube.obj") ;

    for( uint i=0 ; i<10 ; i++ ) {
        Vector4f clr(0.5, g_rng.uniform(0.0, 1.0), g_rng.uniform(0.0, 1.0), 1.0) ;

        scene->addChild(randomBox(format("box%d", i), Material::makeLambertian(clr), Vector3f(0.04, g_rng.uniform(0.1, 0.15), 0.04))) ;
    }

    DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
    dl->diffuse_color_ = Vector3f(1, 1, 1) ;
    scene->addLight(LightPtr(dl)) ;

    QApplication app(argc, argv);

    QSurfaceFormat format;
      format.setDepthBufferSize(24);
      format.setMajorVersion(3);
      format.setMinorVersion(3);

      format.setSamples(4);
      format.setProfile(QSurfaceFormat::CoreProfile);

      QSurfaceFormat::setDefaultFormat(format);

    QMainWindow window ;
    window.setCentralWidget(new ExampleWindow(scene)) ;
    window.show() ;

    return app.exec();
}
