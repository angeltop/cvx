#ifndef __CVX_VIZ_MATERIAL_HPP__
#define __CVX_VIZ_MATERIAL_HPP__

#include <Eigen/Geometry>
#include <cvx/viz/renderer/ogl_shaders.hpp>
#include <cvx/viz/scene/light.hpp>
#include <opencv2/opencv.hpp>

namespace cvx { namespace viz {

class Material ;
using MaterialPtr = std::shared_ptr<Material> ;

// texture and its parameters

class Texture2D {
public:

    Texture2D(const std::string &url): image_url_(url) {}
    Texture2D(const cv::Mat &im): im_(im) {}

    void read() ;
    void upload() ;

private:
    std::string image_url_ ;       // url should be file://<absolute path>
    std::string wrap_s_, wrap_t_ ;
    cv::Mat im_ ;

    uint texture_id_ = 0 ;
};

class MaterialInstance ;
using MaterialInstancePtr = std::shared_ptr<MaterialInstance> ;

// A material encapsulates the technique used render a mesh on screen

class Material {
public:
    Material() = default ;

    // Should be overriden to return the shader program used to render the material.
    // Since this should be called after context creation it should create the required shaders on demand and cache it
    // either in the stock_shaders library or in some internal static variable

    virtual OpenGLShaderProgram::Ptr prog() = 0 ;

    static MaterialInstancePtr makeLambertian(const Eigen::Vector4f &clr) ;
    static MaterialInstancePtr makeConstant(const Eigen::Vector4f &clr) ;

protected:


} ;

// Material paremeters are associated with each material instance, allowing to render several objects using the same technique but different parameters

class MaterialParameters {
public:
    MaterialParameters() = default ;
    virtual ~MaterialParameters() = default ;

protected:

    bool loadTexture(Texture2D &data) ;
};


class MaterialInstance {

public:

    virtual void use() ;
    virtual void applyParameters() {}
    virtual void applyTransform(const Eigen::Matrix4f &cam, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) {}
    virtual void applyLight(uint idx, const LightPtr &light, const Eigen::Isometry3f &tf) {}

protected:
    MaterialInstance(const std::shared_ptr<Material> &material, const std::shared_ptr<MaterialParameters> &params):
        material_(material), params_(params) {}

    void applyDefaultPerspective(const Eigen::Matrix4f &cam, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) ;
    void applyDefaultLight(uint idx, const LightPtr &light, const Eigen::Isometry3f &tf) ;

    std::shared_ptr<Material> material_ ;
    std::shared_ptr<MaterialParameters> params_ ;
};



class ConstantMaterialParameters: public MaterialParameters {
public:

    ConstantMaterialParameters(const Eigen::Vector4f &clr) ;

    void setColor(const Eigen::Vector4f &c) { clr_ = c ; }

protected:

    friend class ConstantMaterialInstance ;

    Eigen::Vector4f clr_ = { 1, 1, 1, 1} ;
} ;


class ConstantMaterialInstance: public MaterialInstance {
public:
    ConstantMaterialInstance(const Eigen::Vector4f &clr) ;
    ConstantMaterialInstance(const std::shared_ptr<ConstantMaterialParameters> &params) ;

protected:
    void applyParameters() override ;

    void applyTransform(const Eigen::Matrix4f &cam, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) override {
        applyDefaultPerspective(cam, view, model) ;
    }

    void applyLight(uint idx, const LightPtr &light, const Eigen::Isometry3f &tf) override {
        applyDefaultLight(idx, light, tf) ;
    }

};

class PhongMaterialParameters: public MaterialParameters {
public:

    PhongMaterialParameters() = default ;

    void setAmbient(const Eigen::Vector4f &a) { ambient_ = a ; }
    void setDiffuse(const Eigen::Vector4f &d) { diffuse_ = d ; }
    void setSpecular(const Eigen::Vector4f &s) { specular_ = s; }
    void setShininess(float s) { shininess_ = s ; }

protected:

    friend class PhongMaterialInstance ;

    Eigen::Vector4f ambient_ = { 0, 0, 0, 1},
    diffuse_ = { 0.5, 0.5, 0.5, 1.0 },
    specular_ = { 0, 0, 0, 1 };
    float shininess_  = 1.0 ;
} ;

class PhongMaterialInstance: public MaterialInstance {
public:
    PhongMaterialInstance() ;
    PhongMaterialInstance(const std::shared_ptr<PhongMaterialParameters> &params) ;

    PhongMaterialParameters &params() {
        return *std::static_pointer_cast<PhongMaterialParameters>(params_) ;
    }
protected:
    void applyParameters() override ;

    void applyTransform(const Eigen::Matrix4f &cam, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) override {
        applyDefaultPerspective(cam, view, model) ;
    }

    void applyLight(uint idx, const LightPtr &light, const Eigen::Isometry3f &tf) override {
        applyDefaultLight(idx, light, tf) ;
    }
};

class DiffuseMapMaterialParameters: public MaterialParameters {
public:

    DiffuseMapMaterialParameters(const Texture2D &tex): diffuse_map_(tex) {}

    void setAmbient(const Eigen::Vector4f &a) { ambient_ = a ; }
    void setDiffuse(const Texture2D &t) { diffuse_map_ = t ; }
    void setSpecular(const Eigen::Vector4f &s) { specular_ = s; }
    void setShininess(float s) { shininess_ = s ; }

    void setTexture(const Texture2D &tex) { diffuse_map_ = tex ; }

protected:

    friend class DiffuseMapMaterialInstance ;

    Eigen::Vector4f ambient_ = { 0, 0, 0, 1},
    specular_ = { 0, 0, 0, 1 };
    float shininess_  = 1.0 ;
    Texture2D diffuse_map_ ;
} ;

class DiffuseMapMaterialInstance: public MaterialInstance {
public:
    DiffuseMapMaterialInstance(const Texture2D &tex) ;
    DiffuseMapMaterialInstance(const std::shared_ptr<DiffuseMapMaterialParameters> &params) ;

    DiffuseMapMaterialParameters &params() {
        return *std::static_pointer_cast<DiffuseMapMaterialParameters>(params_) ;
    }

protected:
    void applyTransform(const Eigen::Matrix4f &cam, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) override {
        applyDefaultPerspective(cam, view, model) ;
    }

    void applyParameters() override ;

    void applyLight(uint idx, const LightPtr &light, const Eigen::Isometry3f &tf) override {
        applyDefaultLight(idx, light, tf) ;
    }
};

// This uses the colors specified per vertex as the ambient component and the specified diffuse color and normals for shading

class PerVertexColorMaterialParameters: public MaterialParameters {
public:

    PerVertexColorMaterialParameters(float o = 1.0): opacity_(o) {}

    void setOpacity(float o) { opacity_ = o ; }

protected:

    friend class PerVertexColorMaterialInstance ;

    float opacity_ = 1.0 ;
} ;

class PerVertexColorMaterialInstance: public MaterialInstance {
public:
    PerVertexColorMaterialInstance(float opac) ;
    PerVertexColorMaterialInstance(const std::shared_ptr<PerVertexColorMaterialParameters> &params) ;

    PerVertexColorMaterialParameters &params() {
        return *std::static_pointer_cast<PerVertexColorMaterialParameters>(params_) ;
    }
protected:
    void applyParameters() override ;

    void applyTransform(const Eigen::Matrix4f &cam, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) override {
        applyDefaultPerspective(cam, view, model) ;
    }

    void applyLight(uint idx, const LightPtr &light, const Eigen::Isometry3f &tf) override {
        applyDefaultLight(idx, light, tf) ;
    }
};

} // namespace viz
} // namespave cvx
#endif
