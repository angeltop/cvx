#include <cvx/viz/scene/camera.hpp>

#include <iostream>
#include <fstream>

#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz {

// http://antongerdelan.net/opengl/raycasting.html

Ray PerspectiveCamera::getRay(float sx, float sy) const
{
    float x = (2.0f * sx) / vp_.width_ - 1.0f;
    float y = 1.0f - (2.0f * sy) / vp_.height_;

    Vector4f ray_clip(x, y, -1, 1) ;

    Vector4f ray_eye = getProjectionMatrix().inverse() * ray_clip;
    ray_eye.z() = -1 ; ray_eye.w() = 0 ;

    Matrix4f mvi = mat_.inverse() ;
    Vector3f origin = mvi.block<3, 1>(0, 3);

    Vector3f ray_world = (mvi * ray_eye).head<3>().normalized() ;

    return Ray(origin, ray_world) ;
}

Eigen::Matrix4f PerspectiveCamera::projectionMatrix() const {
    assert(abs(aspect_ - std::numeric_limits<float>::epsilon()) > static_cast<float>(0));

    float xfov = aspect_ * yfov_ ;
    float const d = 1/tan(xfov / static_cast<float>(2));

    Matrix4f result ;
    result.setZero() ;

    result(0, 0) = d / aspect_ ;
    result(1, 1) = d ;
    result(2, 2) =  (zfar_ + znear_) / (znear_ - zfar_);
    result(2, 3) =  2 * zfar_ * znear_ /(znear_ - zfar_) ;
    result(3, 2) = -1 ;

    return result;
}


void Camera::lookAt(const Vector3f &eye, const Vector3f &center, const Vector3f &up) {
    Vector3f f = (center - eye).normalized();
    Vector3f s = f.cross(up).normalized();
    Vector3f u = s.cross(f) ;

    mat_ << s.x(), s.y(), s.z(), -s.dot(eye),
            u.x(), u.y(), u.z(), -u.dot(eye),
            -f.x(), -f.y(), -f.z(), f.dot(eye),
            0, 0, 0, 1 ;
}

void Camera::lookAt(const Vector3f &eye, const Vector3f &center, float roll) {
    lookAt(eye, center, Vector3f(0, 1, 0)) ;

    Affine3f rot ;
    rot.setIdentity();
    rot.rotate(AngleAxisf(roll, Eigen::Vector3f::UnitZ())) ;
    mat_ = mat_ * rot.matrix() ;
}

Ray::Ray(const Vector3f &orig, const Vector3f &dir) : orig_(orig), dir_(dir) {
    invdir_ = Vector3f(1.0f/dir.x(), 1.0f/dir.y(), 1.0f/dir.z());
    sign_[0] = (invdir_.x() < 0);
    sign_[1] = (invdir_.y() < 0);
    sign_[2] = (invdir_.z() < 0);
}

Ray::Ray(const Ray &other, const Eigen::Isometry3f &tr):
    Ray((tr * other.orig_.homogeneous()).head<3>(), tr.linear() * other.dir_) {}

} // namespace viz
              } // namespace cvx
