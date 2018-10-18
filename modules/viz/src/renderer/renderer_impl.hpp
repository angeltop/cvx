#ifndef __VSIM_RENDERER_IMPL_HPP__
#define __VSIM_RENDERER_IMPL_HPP__

#include <memory>

#include <cvx/viz/scene/scene.hpp>
#include <cvx/viz/renderer/ogl_shaders.hpp>
#include <cvx/viz/renderer/renderer.hpp>

#include "GL/gl3w.h"

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "text_item.hpp"

namespace cvx { namespace viz { namespace impl {

class RendererImpl {
public:

    RendererImpl(const ScenePtr &scene): scene_(scene) {}
    ~RendererImpl() ;

    // initialize renderer
    bool init() ;

    static const int MAX_TEXTURES = 4 ;

    enum VB_TYPES {
        INDEX_BUFFER,
        POS_VB,
        NORMAL_VB,
        COLOR_VB,
        TEXCOORD_VB,
        TF_VB = TEXCOORD_VB + MAX_TEXTURES,
        NUM_VBs
    };

    struct MeshData {
        MeshData() ;
        GLuint buffers_[10] = {0};
        GLuint texture_id_, vao_ ;
        GLuint elem_count_, indices_  ;
    };

    void clear(MeshData &data);
    void initBuffersForMesh(MeshData &data, Mesh &mesh) ;

    void render(const CameraPtr &cam) ;
    void render(const NodePtr &node, const Eigen::Matrix4f &mat) ;
    void render(const DrawablePtr &geom, const Eigen::Matrix4f &mat) ;
    void setModelTransform(const Eigen::Matrix4f &tf);

    void setProgram(const std::string &) ;
    void drawMeshData(MeshData &data, GeometryPtr geom);

    void setLights(const MaterialInstancePtr &mat) ;
    void setLights(const NodePtr &node, const Eigen::Isometry3f &parent_tf, const MaterialInstancePtr &mat) ;

    void initTextures() ;
    void renderText(const std::string &text, float x, float y, const Font &face, const Eigen::Vector3f &clr) ;
    void initFontData() ;
    void makeVertexBuffers();


    cv::Mat getColor(bool alpha);
    cv::Mat getColor(cv::Mat &bg, float alpha);
    cv::Mat getDepth();

private:

    OpenGLShaderLibrary shaders_ ;

    std::map<GeometryPtr, MeshData> buffers_ ;
    std::map<std::string, GLuint> textures_ ;

    std::map<std::string, std::string> user_shaders_ ;
    ScenePtr scene_ ;
    Eigen::Matrix4f perspective_, proj_ ;
    GLuint query_ ;
    Eigen::Vector4f bg_clr_= { 0, 0, 0, 1 } ;
    float znear_, zfar_ ;
    MaterialInstancePtr default_material_ ;
    OpenGLShaderProgram::Ptr prog_ ;
    uint light_index_ = 0 ;

    static detail::GlyphCacheMap g_glyphs ;
} ;


}}}

#endif
