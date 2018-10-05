#include "renderer_impl.hpp"
#include "font_manager.hpp"
#include "glyph_cache.hpp"

#include <iostream>
#include <cstring>

#include <Eigen/Dense>

#include <fstream>
#include <memory>
#include <set>

#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/scene/drawable.hpp>
#include <cvx/viz/scene/mesh.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/material.hpp>
#include <cvx/viz/scene/geometry.hpp>

#include <cvx/util/misc/strings.hpp>

using namespace std ;
using namespace Eigen ;
using namespace cvx::util ;

namespace cvx { namespace viz { namespace impl {

void RendererImpl::makeVertexBuffers() {

    set<GeometryPtr> geometries ;

    scene_->visit([&](Node &n) {
        for( const auto &dr: n.drawables() ) {
            GeometryPtr g = dr->geometry() ;
            if ( g ) geometries.insert(g) ;
        }
    }) ;

    for( const GeometryPtr &geom: geometries)  {
        MeshData data ;
        if ( MeshPtr mesh = std::dynamic_pointer_cast<Mesh>(geom) ) {
            MeshPtr fmesh = Mesh::flatten(mesh) ;
            initBuffersForMesh(data, *fmesh);
            buffers_[geom] = data ;
        } else if ( auto box_geom = std::dynamic_pointer_cast<BoxGeometry>(geom) ) {
            MeshPtr mesh = Mesh::createSolidCube(box_geom->halfExtents()) ;
            MeshPtr fmesh = Mesh::flatten(mesh) ;
            initBuffersForMesh(data, *fmesh);
            buffers_[geom] = data ;
        }
    }
}

void RendererImpl::addTextureImage(const string &id, const cv::Mat &im)
{
    inmemory_textures_.emplace(id, im) ;
}

void GLAPIENTRY
errorCallback( GLenum source,
                 GLenum type,
                 GLuint id,
                 GLenum severity,
                 GLsizei length,
                 const GLchar* message,
                 const void* userParam )
{
  fprintf( stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
           ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
            type, severity, message );
}

bool RendererImpl::init() {



    // setup glew

    glewExperimental = GL_TRUE ;

    GLenum err ;
    if ( ( err = glewInit() ) != GLEW_OK ) {
        return false ; // this may indicate that context has not been initialized
    }

    // During init, enable debug output
    glEnable              ( GL_DEBUG_OUTPUT );
    glDebugMessageCallback( errorCallback, this );
    glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE);
    glDebugMessageControl(GL_DONT_CARE, GL_DEBUG_TYPE_ERROR, GL_DONT_CARE, 0, nullptr, true);

    // create vertex buffers

    makeVertexBuffers() ;

    // load textures

    initTextures() ;

    default_material_.reset(new PhongMaterial) ;



    return true ;
}

void RendererImpl::clear(MeshData &data) {
    glDeleteVertexArrays(1, &data.vao_) ;
}

#define POSITION_LOCATION    0
#define NORMALS_LOCATION    1
#define COLORS_LOCATION    2
#define BONE_ID_LOCATION    3
#define BONE_WEIGHT_LOCATION    4
#define UV_LOCATION 5

RendererImpl::MeshData::MeshData() {}

void RendererImpl::initBuffersForMesh(MeshData &data, Mesh &mesh)
{
    // Create the VAO

    glGenVertexArrays(1, &data.vao_);
    glBindVertexArray(data.vao_);

    const EPointList3f &vertices = mesh.vertices().data() ;
    const EPointList3f &normals = mesh.normals().data() ;
    const EPointList3f &colors = mesh.colors().data() ;

    data.elem_count_ = mesh.vertices().data().size() ;
    data.indices_ = mesh.vertices().indices().size() ;

    glGenBuffers(1, &data.buffers_[POS_VB]);
    glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[POS_VB]);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat) * 3, &vertices[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(POSITION_LOCATION);
    glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    if ( !normals.empty() ) {
        glGenBuffers(1, &data.buffers_[NORMAL_VB]);
        glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[NORMAL_VB]);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(GLfloat) * 3, (GLfloat *)normals.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(NORMALS_LOCATION);
        glVertexAttribPointer(NORMALS_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    }

    if ( !colors.empty() ) {
        glGenBuffers(1, &data.buffers_[COLOR_VB]);
        glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[COLOR_VB]);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat) * 3, (GLfloat *)colors.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(COLORS_LOCATION);
        glVertexAttribPointer(COLORS_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    }

    for( uint t = 0 ; t<MAX_TEXTURES ; t++ ) {
        if ( !mesh.texCoords(t).data().empty() ) {
            glGenBuffers(1, &data.buffers_[TEXCOORD_VB + t]);
            glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[TEXCOORD_VB + t]);
            glBufferData(GL_ARRAY_BUFFER, mesh.texCoords(t).data().size() * sizeof(GLfloat) * 2, (GLfloat *)mesh.texCoords(t).data().data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(UV_LOCATION + t);
            glVertexAttribPointer(UV_LOCATION + t, 2, GL_FLOAT, GL_FALSE, 0, NULL);
        }
    }

#if 0
    glGenBuffers(1, &data.buffers_[TF_VB]);
    glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[TF_VB]);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat) * 3, 0, GL_STATIC_READ);
#endif

    if ( mesh.vertices().hasIndices() ) {
        glGenBuffers(1, &data.buffers_[INDEX_BUFFER]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, data.buffers_[INDEX_BUFFER]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.vertices().indices().size() * sizeof(uint32_t),
                     mesh.vertices().indices().data(), GL_STATIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}



void RendererImpl::render(const CameraPtr &cam) {

    glEnable(GL_DEPTH_TEST) ;
    glDepthFunc(GL_LESS);

    glEnable(GL_CULL_FACE) ;
    glCullFace(GL_BACK) ;
    glFrontFace(GL_CCW) ;

    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    Vector4f bg_clr = cam->bgColor() ;

    glClearColor(bg_clr.x(), bg_clr.y(), bg_clr.z(), bg_clr.w()) ;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    if ( const auto &pcam = dynamic_pointer_cast<PerspectiveCamera>(cam) ) {
        perspective_ = pcam->projectionMatrix() ;
        znear_ = pcam->zNear() ;
        zfar_ = pcam->zFar() ;
    }

    const Viewport &vp = cam->getViewport() ;
    glViewport(vp.x_, vp.y_, vp.width_, vp.height_);

    proj_ = cam->getViewMatrix() ;

    // render node hierarchy

    render(scene_, Matrix4f::Identity()) ;

}

void RendererImpl::render(const NodePtr &node, const Matrix4f &tf) {

    if ( !node->isVisible() ) return ;

    Matrix4f mat = node->matrix().matrix(),
            tr = tf * mat ; // accumulate transform

    for( uint i=0 ; i<node->numDrawables() ; i++ ) {
        const DrawablePtr &m = node->getDrawable(i) ;
        render(m, tr) ;
    }

    for( uint i=0 ; i<node->numChildren() ; i++ ) {
        const NodePtr &n = node->getChild(i) ;
        render(n, tr) ;
    }
}


RendererImpl::~RendererImpl() {

}

void RendererImpl::setModelTransform(const Matrix4f &tf)
{
    Matrix4f mvp =  perspective_ * proj_ * tf;
    Matrix4f mv =   proj_ * tf;
    Matrix3f wp = mv.block<3, 3>(0, 0).transpose().inverse() ;

    prog_->setUniform("mvp", mvp) ;
    prog_->setUniform("mv", mv) ;
    prog_->setUniform("mvn", wp) ;
}

void RendererImpl::setMaterial(const MaterialPtr &material)
{
    material->apply() ;

    vector<Texture2D> textures ;
    material->getTextureData(textures);

    uint s = 0 ;
    for( const Texture2D &t: textures ) {
        glActiveTexture(GL_TEXTURE0 + s++) ;
        glBindTexture(GL_TEXTURE_2D, textures_[t.image_url_] );
    }

}

#define MAX_LIGHTS 10

void RendererImpl::setLights(const NodePtr &node, const Isometry3f &parent_tf)
{
    Isometry3f tf = parent_tf * node->matrix() ;

    for( uint i=0 ; i< node->lights().size() ; i++  ) {

        if ( light_index_ >= MAX_LIGHTS ) return ;

        string vname = format("g_light_source[%d]", light_index_++) ;

        const LightPtr &light = node->lights()[i] ;

        if ( const auto &alight = std::dynamic_pointer_cast<AmbientLight>(light) ) {

            prog_->setUniform(vname + ".light_type", 0) ;
            prog_->setUniform(vname + ".color", alight->color_) ;
        }
        else if ( const auto &dlight = std::dynamic_pointer_cast<DirectionalLight>(light) ) {
            prog_->setUniform(vname + ".light_type", 1) ;
            prog_->setUniform(vname + ".color", dlight->diffuse_color_) ;
            prog_->setUniform(vname + ".direction", tf * dlight->direction_) ;
        }
        else if ( const auto &slight = std::dynamic_pointer_cast<SpotLight>(light) ) {

            prog_->setUniform(vname + ".light_type", 2) ;
            prog_->setUniform(vname + ".color", slight->diffuse_color_) ;
            prog_->setUniform(vname + ".direction", tf * slight->direction_) ;
            prog_->setUniform(vname + ".position", tf * slight->position_) ;
            prog_->setUniform(vname + ".constant_attenuation", slight->constant_attenuation_) ;
            prog_->setUniform(vname + ".linear_attenuation", slight->linear_attenuation_) ;
            prog_->setUniform(vname + ".quadratic_attenuation", slight->quadratic_attenuation_) ;
            prog_->setUniform(vname + ".spot_exponent", slight->falloff_exponent_) ;
            prog_->setUniform(vname + ".spot_cos_cutoff", (float)cos(M_PI*slight->falloff_angle_/180.0)) ;
        }
        else if ( const auto &plight = std::dynamic_pointer_cast<PointLight>(light)) {
            prog_->setUniform(vname + ".light_type", 3) ;
            prog_->setUniform(vname + ".color", plight->diffuse_color_) ;
            prog_->setUniform(vname + ".position", tf * plight->position_) ;
            prog_->setUniform(vname + ".constant_attenuation", plight->constant_attenuation_) ;
            prog_->setUniform(vname + ".linear_attenuation", plight->linear_attenuation_) ;
            prog_->setUniform(vname + ".quadratic_attenuation", plight->quadratic_attenuation_) ;
        }
    }

    for( uint i=0 ; i<node->numChildren() ; i++ )
        setLights(node->getChild(i), tf) ;
}

void RendererImpl::setLights() {
    light_index_ = 0 ;

    Isometry3f mat ;
    mat.setIdentity() ;

    setLights(scene_, mat) ;
}

void RendererImpl::initTextures()
{
    // collect all materials

    set<MaterialPtr> materials ;

    scene_->visit([&](Node &n) {
        for( const auto &dr: n.drawables() ) {
            MaterialPtr m = dr->material() ;
            if ( m ) materials.insert(m) ;
        }
    }) ;

    for( const MaterialPtr &m: materials ) {

        vector<Texture2D> textures ;
        m->getTextureData(textures) ;

        for( Texture2D &texture: textures ) {

            if ( textures_.find(texture.image_url_) != textures_.end() ) continue ;

            cv::Mat image ;
            string ipath ;
            if ( startsWith(texture.image_url_, "mem://" ) ) { // image is in memory indexed by id
                ipath = texture.image_url_.substr(6) ;
                auto it = inmemory_textures_.find(ipath) ;
                if ( it != inmemory_textures_.end() )
                    image = it->second ;
            } else if ( startsWith(texture.image_url_, "file://" ) ) {
                ipath = texture.image_url_.substr(7) ;
                image = cv::imread(ipath) ;
            }

            if ( image.data ) {

                cv::flip(image, image, 0) ;

                GLuint texture_id ;

                glActiveTexture(GL_TEXTURE0);
                glGenTextures(1, &texture_id);
                glBindTexture(GL_TEXTURE_2D, texture_id);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

                // Set texture clamping method
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

                glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
                glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
                glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0 );
                glPixelStorei( GL_UNPACK_SKIP_ROWS, 0 );

                glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                             0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                             GL_RGB,            // Internal colour format to convert to
                             image.cols,
                             image.rows,
                             0,                 // Border width in pixels (can either be 1 or 0)
                             GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                             GL_UNSIGNED_BYTE,  // Image data type
                             image.ptr());        // The actual image data itself

                glGenerateMipmap(GL_TEXTURE_2D);

                textures_[texture.image_url_] = texture_id ;
            }

        }

    }
}



void RendererImpl::drawMeshData(MeshData &data, GeometryPtr geom) {
    glBindVertexArray(data.vao_);

    MeshPtr mesh = std::dynamic_pointer_cast<Mesh>(geom) ;

    if ( mesh ) {
        if ( mesh->ptype() == Mesh::Triangles ) {
            if ( mesh->vertices().hasIndices() ) {
                // bind index buffer if you want to render indexed data
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, data.buffers_[INDEX_BUFFER]);
                // indexed draw call
                glDrawElements(GL_TRIANGLES, data.indices_, GL_UNSIGNED_INT, NULL);
            }
            else
                glDrawArrays(GL_TRIANGLES, 0, data.elem_count_) ;
        }
        else if ( mesh->ptype() == Mesh::Lines ) {
            glDrawArrays(GL_LINES, 0, data.elem_count_) ;
        }
        else if ( mesh->ptype() == Mesh::Points ) {
            glDrawArrays(GL_POINTS, 0, data.elem_count_) ;
        }

    } else {
        glDrawArrays(GL_TRIANGLES, 0, data.elem_count_) ;
    }

    glBindVertexArray(0) ;

    glFlush();
}


void RendererImpl::render(const DrawablePtr &geom, const Matrix4f &mat)
{
    if ( !geom->geometry() ) return ;

    MeshData &data = buffers_[geom->geometry()] ;

    // prog_ = shaders_.get(mode) ;

    MaterialPtr material = geom->material() ;
    if ( !material ) material = default_material_ ;

    OpenGLShaderProgram::Ptr old_prog = prog_ ;

    prog_ = material->prog() ;

    assert( prog_ ) ;

    //    if ( prog_ != old_prog )
    prog_->use() ;

    setModelTransform(mat) ;

    setMaterial(material) ;

    setLights() ;

#if 0

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, data.buffers_[TF_VB]);

    glEnable(GL_RASTERIZER_DISCARD);

    glBeginTransformFeedback(GL_TRIANGLES);

    drawMeshData(data, geom->geometry()) ;

    glEndTransformFeedback();

    glDisable(GL_RASTERIZER_DISCARD);

    vector<GLfloat> fdata(36, 0) ;

    glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 36*sizeof(GLfloat), fdata.data());

    glBindVertexArray(0) ;
#else
    drawMeshData(data, geom->geometry()) ;

#endif
    // glUseProgram(0) ;
}


cv::Mat RendererImpl::getColor(bool alpha)
{
    GLint v[4] ;
    glGetIntegerv(GL_VIEWPORT, v) ;

    uint width = v[2], height = v[3] ;

    if ( alpha )
    {
        cv::Mat_<cv::Vec4b> mask(height, width) ;
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, mask.ptr());

        cv::flip(mask, mask, 0) ;

        cv::cvtColor(mask, mask, CV_RGBA2BGRA);

        return mask ;
    }
    else
    {
        cv::Mat_<cv::Vec3b> mask(height, width) ;
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, mask.ptr());

        cv::flip(mask, mask, 0) ;

        cv::cvtColor(mask, mask, CV_RGB2BGR);

        return mask ;
    }
}

cv::Mat RendererImpl::getColor(cv::Mat &bg, float alpha)
{
    GLint v[4] ;
    glGetIntegerv(GL_VIEWPORT, v) ;

    uint width = v[2], height = v[3] ;

    cv::Mat_<cv::Vec3b> mask(height, width) ;
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, mask.ptr());
    cv::flip(mask, mask, 0) ;
    cv::cvtColor(mask, mask, CV_RGB2BGR);

    cv::Mat dst ;
    cv::addWeighted( mask, alpha, bg, (1 - alpha), 0.0, dst);

    return dst ;
}

cv::Mat RendererImpl::getDepth() {

    GLint v[4] ;
    glGetIntegerv(GL_VIEWPORT, v) ;

    uint width = v[2], height = v[3] ;

    cv::Mat_<float> depth(height, width);

    glReadBuffer(GL_DEPTH_ATTACHMENT);

    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

    cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
    float max_allowed_z = zfar_ * 0.99;


    for (unsigned int j = 0; j < height; ++j)
        for (unsigned int i = 0; i < width; ++i, ++it)
        {
            //need to undo the depth buffer mapping
            //http://olivers.posterous.com/linear-depth-in-glsl-for-real
            float z  = 2 * zfar_ * znear_ / (zfar_ + znear_ - (zfar_ - znear_) * (2 * (*it) - 1));

            if (z > max_allowed_z) *it = 0;
            else {
                *it = z ;
            }
        }
    // Rescale the depth to be in millimeters
    cv::Mat depth_scale(cv::Size(width, height), CV_16UC1);
    depth.convertTo(depth_scale, CV_16UC1, 1e3);

    cv::flip(depth_scale, depth_scale, 0) ;

    // cv::imwrite("/tmp/dmap.png", vl::depthViz(depth_scale)) ;
    return depth_scale;

}


void RendererImpl::renderText(const string &text, float x, float y, const Font &font, const Vector3f &clr) {
    using namespace detail ;

    if ( text.empty() ) return ;

    TextItem ti(text, font) ;
    ti.render(x, y, clr) ;
}

}}}
