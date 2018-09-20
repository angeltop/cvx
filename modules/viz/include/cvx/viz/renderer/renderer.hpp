#ifndef __VSIM_RENDERER_HPP__
#define __VSIM_RENDERER_HPP__

#include <memory>

#include <cvx/viz/scene/scene.hpp>
#include <cvx/viz/gui/offscreen.hpp>
#include <opencv2/opencv.hpp>

namespace cvx { namespace viz {

namespace impl {
    class RendererImpl ;
}

class Renderer {
public:

    Renderer(const ScenePtr &scene) ;
    ~Renderer() ;

    // initialize renderer
    bool init() ;

    // set free space color
    void setBackgroundColor(const Eigen::Vector4f &clr);

    // render scene
    void render(const CameraPtr &cam) ;

    cv::Mat getColor(bool alpha = true);
    cv::Mat getColor(cv::Mat &bg, float alpha);
    cv::Mat getDepth();

    // Add an in-memory image (the associated material should reference the texture by mem://<id>
    // It should be called before calling init.

    void addTextureImage(const std::string &id, const cv::Mat &im) ;

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
