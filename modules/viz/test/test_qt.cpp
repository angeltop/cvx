#include <GL/glew.h>

#include <QtGui/QtGui>
#include <QApplication>
#include <QMainWindow>
#include "qt_glwidget.hpp"



#include <cvx/viz/renderer/renderer.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/gui/trackball.hpp>

#include <iostream>

using namespace Eigen ;

using namespace std ;
using namespace cvx::viz ;


class AABBox
{
    public:
    AABBox(const Vector3f &b0, const Vector3f &b1) {
        bounds[0] = b0 ;
        bounds[1] = b1;
    }

    bool intersect(const Ray &r, float &t) const  {
        float tmin, tmax, tymin, tymax, tzmin, tzmax;

        tmin = (bounds[r.sign_[0]].x() - r.orig_.x()) * r.invdir_.x();
        tmax = (bounds[1-r.sign_[0]].x() - r.orig_.x()) * r.invdir_.x();
        tymin = (bounds[r.sign_[1]].y() - r.orig_.y()) * r.invdir_.y();
        tymax = (bounds[1-r.sign_[1]].y() - r.orig_.y()) * r.invdir_.y();

        if ((tmin > tymax) || (tymin > tmax)) return false;

        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        tzmin = (bounds[r.sign_[2]].z() - r.orig_.z()) * r.invdir_.z();
        tzmax = (bounds[1-r.sign_[2]].z() - r.orig_.z()) * r.invdir_.z();

        if ((tmin > tzmax) || (tzmin > tmax)) return false;

        if (tzmin > tmin) tmin = tzmin;
        if (tzmax < tmax) tmax = tzmax;

        t = tmin ;
        if ( t < 0 ) {
            t = tmax ;
            if ( t < 0 ) return false;
        }

        return true;
    }

    Vector3f bounds[2];
};

struct Triangle {
    Vector3f vtx_[3] ;
};

bool rayIntersectsTriangle(const Ray ray,
                           const Vector3f &v0,
                           const Vector3f &v1,
                           const Vector3f &v2,
                           bool back_face_culling,
                           float &t)
{
    const float eps = 0.0000001;
    Vector3f edge1, edge2, h, s, q;
    float a,f,u,v;
    edge1 = v1 - v0;
    edge2 = v2 - v0;
    h = ray.dir_.cross(edge2);
    a = edge1.dot(h);
    f = 1/a;

    if ( back_face_culling ) {
        if ( a < eps ) return false;
        s = ray.orig_ - v0;
        u =  ( s.dot(h) );
        if ( u < 0.0 || u > a ) return false;
        q = s.cross(edge1);
        v = ray.dir_.dot(q) ;
        if (v < 0.0 || u + v > a) return false;

        // At this stage we can compute t to find out where the intersection point is on the line.
        t = f * edge2.dot(q) ;
    }
    else {
        if ( a > -eps && a < eps ) return false;
        s = ray.orig_ - v0;
        u = f * ( s.dot(h) );
        if ( u < 0.0 || u > 1.0 ) return false;
        q = s.cross(edge1);
        v = f * ray.dir_.dot(q) ;
        if (v < 0.0 || u + v > 1.0)
            return false;

        // At this stage we can compute t to find out where the intersection point is on the line.
        t = f * edge2.dot(q);

    }

    return t > eps ;

}

bool rayIntersectsSphere(const Ray &ray, const Vector3f &center, float radius, float &t) {

    float t0, t1; // solutions for t if the ray intersects

    float radius2 = radius * radius ;
    Vector3f L = center - ray.orig_;
    float tca = L.dot(ray.dir_);
    // if (tca < 0) return false;
    float d2 = L.dot(L) - tca * tca;
    if (d2 > radius2) return false;
    float thc = sqrt(radius2 - d2);
    t0 = tca - thc;
    t1 = tca + thc;

    if (t0 > t1) std::swap(t0, t1);

    t = t0 ;
    if ( t < 0 ) {
        t = t1; // if t0 is negative, let's use t1 instead
        if ( t < 0 ) return false; // both t0 and t1 are negative
    }

    return true;

}

class ExampleWindow : public GLWidget
{
public:
    ExampleWindow(ScenePtr scene);

    void hit(int mouse_x, int mouse_y) {

        Ray ray = camera_->getRay(mouse_x, mouse_y);
/*
        AABBox box(  {-0.5, -0.5, -0.5}, {0.5, 0.5, 0.5}) ;

        float t ;
        if ( box.intersect(ray, t) ) {
                cout << ray.orig_ + t * ray.dir_ << endl ;
        }
        */

        float t ;
        if ( rayIntersectsTriangle(ray, { -0.5, -0.5, 0.5 }, {  0.5, -0.5, 0.5 }, { -0.5, 0.5, 0.5}, false, t ) ) {
                cout << ray.orig_ + t * ray.dir_ << endl ;
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



     auto c = scene->geomCenter() ;
     auto r = scene->geomRadius(c) ;



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
     scene->load("/home/malasiot/Downloads/triangle.obj") ;

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
