#include <cvx/viz/renderer/renderer.hpp>

#include "renderer_impl.hpp"

#define GL_GLEXT_PROTOTYPES
#include <GL/glew.h>

#include <iostream>
#include <cstring>

#include <Eigen/Dense>

#include <fstream>
#include <memory>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz {

bool Renderer::init() {
    return impl_->init() ;
}

void Renderer::render(const CameraPtr &cam) {
    impl_->render(cam) ;
}


Renderer::Renderer(const ScenePtr &scene): impl_(new impl::RendererImpl(scene)) {
}

Renderer::~Renderer() {
}

cv::Mat Renderer::getColor(bool alpha) {
    return impl_->getColor(alpha) ;
}

cv::Mat Renderer::getColor(cv::Mat &bg, float alpha) {
    return impl_->getColor(bg, alpha) ;
}

cv::Mat Renderer::getDepth() {
    return impl_->getDepth() ;
}

void Renderer::addTextureImage(const string &id, const cv::Mat &im) {
    impl_->addTextureImage(id, im) ;
}

void Renderer::text(const string &text, float x, float y, const Font &f, const Vector3f &clr)
{
    impl_->renderText(text, x, y, f, clr) ;
}

Text Renderer::text(const string &text, const Font &f)
{
    return Text(text, f) ;
}


}}
