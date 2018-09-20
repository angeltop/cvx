#ifndef __CVX_VIZ_MATERIAL_HPP__
#define __CVX_VIZ_MATERIAL_HPP__

#include <Eigen/Core>
#include <cvx/viz/renderer/ogl_shaders.hpp>

namespace cvx { namespace viz {

struct Material ;
typedef std::shared_ptr<Material> MaterialPtr ;

// texture and its parameters
struct Texture2D {
    std::string image_url_ ;       // url should be file://<absolute path> or mem://<id>
    std::string wrap_s_, wrap_t_ ;
};

class Material {
public:
    Material() = default ;

    // Should be overriden to return the shader program used to render the material.
    // Since this should be called after context creation it should create the required shaders on demand and cache it
    // either in the stock_shaders library or in some internal static variable

    virtual OpenGLShaderProgram::Ptr prog() const = 0 ;

    // Ovveride this to feed the correponding shader program with need uniforms

    virtual void apply() = 0 ;

    // list of textures bound to this material (e.g. for materials doing texture mapping)

    virtual void getTextureData(std::vector<Texture2D> &t) { }

    void addTexture(const Texture2D &t) ;

    static MaterialPtr makeLambertian(const Eigen::Vector4f &clr) ;
    static MaterialPtr makeConstant(const Eigen::Vector4f &clr) ;

protected:

    std::vector<Texture2D> textures_ ;
} ;

class ConstantMaterial: public Material {
public:

    ConstantMaterial(const Eigen::Vector4f &clr): clr_(clr) {}

    void setColor(const Eigen::Vector4f &c) { clr_ = c ; }

    OpenGLShaderProgram::Ptr prog() const override ;
    void apply() override ;

protected:

    Eigen::Vector4f clr_ = { 1, 1, 1, 1} ;
} ;


class PhongMaterial: public Material {
public:

    PhongMaterial() = default ;

    void setAmbient(const Eigen::Vector4f &a) { ambient_ = a ; }
    void setDiffuse(const Eigen::Vector4f &d) { diffuse_ = d ; }
    void setSpecular(const Eigen::Vector4f &s) { specular_ = s; }
    void setShininess(float s) { shininess_ = s ; }

    OpenGLShaderProgram::Ptr prog() const override ;
    void apply() override ;

protected:

    Eigen::Vector4f ambient_ = { 0, 0, 0, 1},
    diffuse_ = { 0.5, 0.5, 0.5, 1.0 },
    specular_ = { 0, 0, 0, 1 };
    float shininess_  = 1.0 ;

private:
    static void initShaders() ;
    static OpenGLShaderProgram::Ptr prog_ ;
} ;


class DiffuseMapMaterial: public Material {
public:

    DiffuseMapMaterial() = default ;

    void setAmbient(const Eigen::Vector4f &a) { ambient_ = a ; }
    void setDiffuse(const Texture2D &t) { diffuse_map_ = t ; }
    void setSpecular(const Eigen::Vector4f &s) { specular_ = s; }
    void setShininess(float s) { shininess_ = s ; }

    OpenGLShaderProgram::Ptr prog() const override ;
    void apply() override ;
    void getTextureData(std::vector<Texture2D> &t) override {
        t.push_back(diffuse_map_) ;
    }

protected:

    Eigen::Vector4f ambient_ = { 0, 0, 0, 1},
    specular_ = { 0, 0, 0, 1 };
    float shininess_  = 1.0 ;
    Texture2D diffuse_map_ ;
} ;

// This uses the colors specified per vertex as the ambient component and the specified diffuse color and normals for shading

class PerVertexColorMaterial: public Material {
public:

    PerVertexColorMaterial(float o = 1.0): opacity_(o) {}

    void setOpacity(float o) { opacity_ = o ; }

    OpenGLShaderProgram::Ptr prog() const override ;
    void apply() override ;

protected:

    float opacity_ = 1.0 ;
} ;

} // namespace viz
} // namespave cvx
#endif
