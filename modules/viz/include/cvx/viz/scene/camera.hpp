#ifndef __CVX_VIZ_CAMERA_HPP__
#define __CVX_VIZ_CAMERA_HPP__

#include <string>
#include <vector>
#include <memory>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cvx/util/camera/camera.hpp>


namespace cvx { namespace viz {

// Abstract camera

struct Viewport {
    Viewport() = default ;
    size_t x_ = 0, y_ = 0, width_, height_ ;
};

class Ray
{
    public:
    Ray(const Eigen::Vector3f &orig, const Eigen::Vector3f &dir);

    Ray(const Ray &other, const Eigen::Isometry3f &tr) ;

    Eigen::Vector3f orig_, dir_;
    Eigen::Vector3f invdir_ ;
    int sign_[3];
};

class Camera {
public:

    Camera():  mat_(Eigen::Matrix4f::Identity()) {}
    virtual ~Camera() {}

    void setViewport(size_t w, size_t h) {
        vp_.width_ = w ; vp_.height_ = h ;
    }

    void setViewport(size_t x, size_t y, size_t w, size_t h) {
        vp_.width_ = w ; vp_.height_ = h ;
        vp_.x_ = x ; vp_.y_ = y ;
    }

    void setViewTransform(const Eigen::Matrix4f &vt) {
        mat_ = vt ;
    }

    void setBgColor(const Eigen::Vector4f &clr) {
        bg_clr_ = clr ;
    }

    virtual Ray getRay(float x, float y) const = 0 ;

    virtual Eigen::Matrix4f getProjectionMatrix() const = 0;

    Eigen::Matrix4f getViewMatrix() const { return mat_ ; }
    const Viewport &getViewport() const { return vp_ ; }
    Eigen::Vector4f bgColor() const { return bg_clr_ ; }

    void lookAt(const Eigen::Vector3f &eye, const Eigen::Vector3f &center, float roll = 0);
    void lookAt(const Eigen::Vector3f &eye, const Eigen::Vector3f &center, const Eigen::Vector3f &up);
protected:

    Eigen::Matrix4f mat_ ; // view transformation
    Viewport vp_ ;
    Eigen::Vector4f bg_clr_ = {0, 0, 0, 1} ;
};

// Perspective camera

class PerspectiveCamera: public Camera {
public:
    PerspectiveCamera(float aspect, float yfov, float znear = 0.01, float zfar = 10.0):
        aspect_(aspect), yfov_(yfov), znear_(znear), zfar_(zfar) {
    }

    PerspectiveCamera(const cvx::util::PinholeCamera &cam, float znear = 0.01, float zfar = 10.0): znear_(znear), zfar_(zfar) {
        yfov_ = 2 * atan( cam.sz().height / cam.fy()/2.0)  ;
     //   aspect_ = cam.sz().width / (float) cam.sz().height ;
        aspect_ = 1.0 ;
        vp_.width_ = cam.sz().width ;
        vp_.height_ = cam.sz().height ;
    }

    void setAspectRatio(float asp) {
        aspect_ = asp ;
    }

    Eigen::Matrix4f getProjectionMatrix() const override {
        return projectionMatrix() ;
    }

    Ray getRay(float x, float y) const override ;

    Eigen::Matrix4f projectionMatrix() const ;
    float zNear() const { return znear_ ; }
    float zFar() const { return zfar_ ; }

protected:

    float yfov_, aspect_, znear_, zfar_ ;
};

// Orthographic camera

class OrthographicCamera: public Camera {

    OrthographicCamera() {}

    Ray getRay(float x, float y) const override ;
protected:
    float xmag_, ymag_, znear_, zfar_ ;


};


} // namespace viz
} // namespace cvx

#endif
