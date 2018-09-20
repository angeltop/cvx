#include <cvx/viz/renderer/renderer.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/material.hpp>
#include <cvx/viz/scene/geometry.hpp>
#include <cvx/viz/scene/node.hpp>

#include <cvx/viz/gui/offscreen.hpp>
#include <cvx/util/math/rng.hpp>
#include <cvx/util/misc/strings.hpp>

#include <iostream>

using namespace cvx::viz ;
using namespace cvx::util ;

using namespace std ;
using namespace Eigen ;

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


static string vs_shader =
R"(
#version 330

layout (location = 0) in vec3 vposition;
layout (location = 5) in vec2 vuv;

out vec2 uv ;

uniform mat4 mvp;
uniform mat4 mv;
uniform mat3 mvn ;

void main()
{
    vec4 posl    = vec4(vposition, 1.0);
    gl_Position  = mvp * posl;
    uv = vuv ;
}
)";

string fs_shader =
R"(
#version 330

in vec2 uv;

out vec4 FragColor;
//
// GLSL textureless classic 2D noise "cnoise",
// with an RSL-style periodic variant "pnoise".
// Author:  Stefan Gustavson (stefan.gustavson@liu.se)
// Version: 2011-08-22
//
// Many thanks to Ian McEwan of Ashima Arts for the
// ideas for permutation and gradient selection.
//
// Copyright (c) 2011 Stefan Gustavson. All rights reserved.
// Distributed under the MIT license. See LICENSE file.
// https://github.com/stegu/webgl-noise
//

vec4 mod289(vec4 x)
{
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x)
{
  return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

vec2 fade(vec2 t) {
  return t*t*t*(t*(t*6.0-15.0)+10.0);
}

// Classic Perlin noise
float cnoise(vec2 P)
{
  vec4 Pi = floor(P.xyxy) + vec4(0.0, 0.0, 1.0, 1.0);
  vec4 Pf = fract(P.xyxy) - vec4(0.0, 0.0, 1.0, 1.0);
  Pi = mod289(Pi); // To avoid truncation effects in permutation
  vec4 ix = Pi.xzxz;
  vec4 iy = Pi.yyww;
  vec4 fx = Pf.xzxz;
  vec4 fy = Pf.yyww;

  vec4 i = permute(permute(ix) + iy);

  vec4 gx = fract(i * (1.0 / 41.0)) * 2.0 - 1.0 ;
  vec4 gy = abs(gx) - 0.5 ;
  vec4 tx = floor(gx + 0.5);
  gx = gx - tx;

  vec2 g00 = vec2(gx.x,gy.x);
  vec2 g10 = vec2(gx.y,gy.y);
  vec2 g01 = vec2(gx.z,gy.z);
  vec2 g11 = vec2(gx.w,gy.w);

  vec4 norm = taylorInvSqrt(vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11)));
  g00 *= norm.x;
  g01 *= norm.y;
  g10 *= norm.z;
  g11 *= norm.w;

  float n00 = dot(g00, vec2(fx.x, fy.x));
  float n10 = dot(g10, vec2(fx.y, fy.y));
  float n01 = dot(g01, vec2(fx.z, fy.z));
  float n11 = dot(g11, vec2(fx.w, fy.w));

  vec2 fade_xy = fade(Pf.xy);
  vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
  float n_xy = mix(n_x.x, n_x.y, fade_xy.y);
  return 2.3 * n_xy;
}

// Classic Perlin noise, periodic variant
float pnoise(vec2 P, vec2 rep)
{
  vec4 Pi = floor(P.xyxy) + vec4(0.0, 0.0, 1.0, 1.0);
  vec4 Pf = fract(P.xyxy) - vec4(0.0, 0.0, 1.0, 1.0);
  Pi = mod(Pi, rep.xyxy); // To create noise with explicit period
  Pi = mod289(Pi);        // To avoid truncation effects in permutation
  vec4 ix = Pi.xzxz;
  vec4 iy = Pi.yyww;
  vec4 fx = Pf.xzxz;
  vec4 fy = Pf.yyww;

  vec4 i = permute(permute(ix) + iy);

  vec4 gx = fract(i * (1.0 / 41.0)) * 2.0 - 1.0 ;
  vec4 gy = abs(gx) - 0.5 ;
  vec4 tx = floor(gx + 0.5);
  gx = gx - tx;

  vec2 g00 = vec2(gx.x,gy.x);
  vec2 g10 = vec2(gx.y,gy.y);
  vec2 g01 = vec2(gx.z,gy.z);
  vec2 g11 = vec2(gx.w,gy.w);

  vec4 norm = taylorInvSqrt(vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11)));
  g00 *= norm.x;
  g01 *= norm.y;
  g10 *= norm.z;
  g11 *= norm.w;

  float n00 = dot(g00, vec2(fx.x, fy.x));
  float n10 = dot(g10, vec2(fx.y, fy.y));
  float n01 = dot(g01, vec2(fx.z, fy.z));
  float n11 = dot(g11, vec2(fx.w, fy.w));

  vec2 fade_xy = fade(Pf.xy);
  vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
  float n_xy = mix(n_x.x, n_x.y, fade_xy.y);
  return 2.3 * n_xy;
}

void main (void)
{
    float sn = cnoise(uv) ;
    FragColor = vec4(vec3(0.5, 1.0, 1.0) + vec3(sn, sn, sn), 1) ;
}

)" ;

class PerlinNoiseMaterial: public Material {
public:
    OpenGLShaderProgram::Ptr prog() const override {
        static OpenGLShaderProgram::Ptr prog_ ;

        if ( prog_ ) return prog_ ;

        OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, vs_shader, "vs_shader")) ;
        OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, fs_shader, "fs_shader"))  ;

        prog_.reset(new OpenGLShaderProgram) ;
        prog_->addShader(vs) ;
        prog_->addShader(fs) ;

        prog_->link() ;

        return prog_ ;

    }
    void apply() override {}


} ;

static MaterialPtr custom_material(new PerlinNoiseMaterial) ;

NodePtr randomBox(const string &name, const Vector3f &hs, const Vector4f &clr) {

    NodePtr box_node(new Node) ;
    box_node->setName(name) ;

    GeometryPtr geom(new BoxGeometry(hs)) ;

    DrawablePtr dr(new Drawable(geom, custom_material)) ;

    box_node->addDrawable(dr) ;

    box_node->matrix() = getRandTransform(0) ;

    return box_node ;
}

int main(int argc, char *argv[]) {

    // load scene

    ScenePtr scene(new Scene) ;

    for( uint i=0 ; i<200 ; i++ ) {
        scene->addChild(randomBox(format("box%d", i),
                                  Vector3f(0.04, g_rng.uniform(0.1, 0.15), 0.04),
                                  Vector4f(0.5, g_rng.uniform(0.0, 1.0), g_rng.uniform(0.0, 1.0), 1.0))) ;
    }

        // add a light source

    DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
    dl->diffuse_color_ = Vector3f(0, 1, 0) ;
    scene->addLight(LightPtr(dl)) ;

    // create a camera
    uint width = 640, height = 480 ;
    PerspectiveCamera *pcam = new PerspectiveCamera(width / (float) height, // aspect ratio
                                                    50*M_PI/180,   // fov
                                                    0.0001,        // zmin
                                                    10           // zmax
                                                    ) ;

    CameraPtr cam(pcam) ;

    // position camera to look at the center of the object

    pcam->lookAt({0, 2, 0}, {0, 0, 0}, {0, 0, 1}) ;

    // set camera viewpot

    pcam->setViewport(width, height)  ;

    pcam->setBgColor({1, 1, 1, 1}) ;



    // initialize renderer

    OffscreenRenderer rdr(width, height, scene) ;

    rdr.init() ;

    rdr.render(cam) ;

    // obtain the color buffer
    cv::Mat clr = rdr.getColor() ;

    cv::imwrite("/tmp/oo.png", clr) ;

}

