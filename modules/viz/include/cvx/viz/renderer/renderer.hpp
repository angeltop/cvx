#ifndef __VSIM_RENDERER_HPP__
#define __VSIM_RENDERER_HPP__

#include <memory>

#include <cvx/viz/scene/scene.hpp>
#include <cvx/viz/gui/offscreen.hpp>
#include <cvx/viz/renderer/font.hpp>
#include <cvx/viz/renderer/text.hpp>
#include <opencv2/opencv.hpp>

namespace cvx { namespace viz {

namespace impl {
    class RendererImpl ;
}

class Renderer {
public:

    Renderer(const ScenePtr &scene) ;
    ~Renderer() ;

    // render scene
    void render(const CameraPtr &cam) ;

    cv::Mat getColor(bool alpha = true);
    cv::Mat getColor(cv::Mat &bg, float alpha);
    cv::Mat getDepth();

    // Add an in-memory image (the associated material should reference the texture by mem://<id>
    // It should be called before calling init.

    void addTextureImage(const std::string &id, const cv::Mat &im) ;

    // Draws text on top of the scene using given font and color
    void text(const std::string &text, float x, float y, const Font &f, const Eigen::Vector3f &clr) ;

    // It returns a text object that may be cached and drawn several times by calling render function.
    // It uses OpenGL so it should be called after calling init
    Text text(const std::string &text, const Font &f) ;

private:

    std::unique_ptr<impl::RendererImpl> impl_ ;

} ;

class OffscreenRenderer: public Renderer {
public:
    OffscreenRenderer(uint width, uint height, const ScenePtr &scene): Renderer(scene), ow_(width, height) {}

protected:
    OffscreenRenderingWindow ow_ ;
} ;

}}

#endif
