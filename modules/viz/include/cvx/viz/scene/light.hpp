#ifndef __CVX_VIZ_LIGHT_HPP__
#define __CVX_VIZ_LIGHT_HPP__

#include <string>
#include <vector>
#include <memory>
#include <map>

#include <Eigen/Core>

namespace cvx {


// Abstract light

struct Light {
    Light() = default ;
    virtual ~Light() {}

    std::string name_ ;
};

struct AmbientLight: public Light {
    AmbientLight() {}
    AmbientLight(const Eigen::Vector3f &clr): color_(clr) {}

    Eigen::Vector3f color_ ;
};

struct PointLight: public Light {
    PointLight(const Eigen::Vector3f &pos):  position_(pos) {}

    Eigen::Vector3f position_ ;
    Eigen::Vector3f diffuse_color_, specular_color_, ambient_color_ ;
    float constant_attenuation_ = 1.0 ;
    float linear_attenuation_ = 0.0;
    float quadratic_attenuation_ = 0.0;
};

struct DirectionalLight: public Light {
    DirectionalLight(const Eigen::Vector3f &dir): direction_(dir) {}

    Eigen::Vector3f direction_ ;
    Eigen::Vector3f diffuse_color_, specular_color_, ambient_color_ ;
};

struct SpotLight: public Light {

    SpotLight(const Eigen::Vector3f &pos, const Eigen::Vector3f &dir):
       position_(pos), direction_(dir) {}

    Eigen::Vector3f direction_, position_ ;
    Eigen::Vector3f diffuse_color_, specular_color_, ambient_color_ ;
    float constant_attenuation_ = 1.0;
    float linear_attenuation_ = 0.0;
    float quadratic_attenuation_ = 0.0;
    float falloff_angle_ = M_PI, falloff_exponent_ = 0 ;
};

} // namespace cvx
#endif
