#include <cvx/viz/scene/material.hpp>
#include <cvx/util/misc/strings.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <GL/gl3w.h>

using namespace std ;
using namespace Eigen ;
using namespace cvx::util ;

static string pn_vertex_shader_code =
        R"(
        #version 330

        layout (location = 0) in vec3 vposition;
        layout (location = 1) in vec3 vnormal;

        out vec3 position;
        out vec3 normal;

        uniform mat4 mvp;
        uniform mat4 mv;
        uniform mat3 mvn ;

        void main()
        {
        vec4 posl    = vec4(vposition, 1.0);
        gl_Position  = mvp * posl;
        normal       = mvn * vnormal;
        position    = (mv * posl).xyz;
        }
        )";

static string pnt_vertex_shader_code =
        R"(
        #version 330 core

        layout (location = 0) in vec3 vposition;
        layout (location = 1) in vec3 vnormal;
        layout (location = 5) in vec2 vuv;

        out vec3 position;
        out vec3 normal;
        out vec2 uv;

        uniform mat4 mvp;
        uniform mat4 mv;
        uniform mat3 mvn ;

        void main()
        {
        vec4 posl    = vec4(vposition, 1.0);
        gl_Position  = mvp * posl;
        normal       = mvn * vnormal;
        position    = (mv * posl).xyz;
        uv = vuv ;
        }
        )";

static string p_vertex_shader_code =
        R"(
        #version 330

        layout (location = 0) in vec3 vposition;

        uniform mat4 mvp;
        uniform mat4 mv;
        uniform mat3 mvn ;

        void main()
        {
        vec4 posl    = vec4(vposition, 1.0);
        gl_Position  = mvp * posl;
        }
        )";

static string pc_vertex_shader_code =
        R"(
        #version 330

        layout (location = 0) in vec3 vposition;
        layout (location = 2) in vec3 vcolor;

        out vec3 color ;

        uniform mat4 mvp;
        uniform mat4 mv;
        uniform mat3 mvn ;

        void main()
        {
        vec4 posl    = vec4(vposition, 1.0);
        gl_Position  = mvp * posl;
        color = vcolor ;
        }
        )";

static string phong_fragment_shader_common =
        R"(
        #version 330
        precision mediump float;
        in vec3 normal;
        in vec3 position;

        const int MAX_LIGHTS = 10;
        const int AMBIENT_LIGHT = 0 ;
        const int DIRECTIONAL_LIGHT = 1 ;
        const int SPOT_LIGHT = 2 ;
        const int POINT_LIGHT = 3 ;

        struct LightSourceParameters
        {
        int light_type ;
        vec3 color;
        vec4 position;
        vec3 direction;
        float spot_exponent;
        float spot_cos_cutoff;
        float constant_attenuation;
        float linear_attenuation;
        float quadratic_attenuation;
        };

        uniform LightSourceParameters g_light_source[MAX_LIGHTS];

        struct MaterialParameters
        {
        vec4 ambient;     // Acm
        vec4 diffuse;     // Dcm
        vec4 specular;    // Scm
        float shininess;  // Srm
        };

        uniform MaterialParameters g_material;

        out vec4 FragColor;

        vec4 phongIllumination(vec4 dc) {
        vec3 N = normalize(normal);
        vec4 finalColor = vec4(0, 0, 0, 1.0);

        for (int i=0;i<MAX_LIGHTS;i++)
        {
        vec3 L ;
        float att = 1.0 ;

        if ( g_light_source[i].light_type == -1 ) continue ;
        else if ( g_light_source[i].light_type == AMBIENT_LIGHT ) {
        finalColor += vec4(g_light_source[i].color, 1.0) * g_material.ambient ;
        continue ;
        }
        else if ( g_light_source[i].light_type == DIRECTIONAL_LIGHT ) {
        L = normalize(g_light_source[i].direction) ;
        }
        else if ( g_light_source[i].light_type == SPOT_LIGHT ) {
        float dist = length(g_light_source[i].position.xyz - position) ;
        L = normalize(g_light_source[i].position.xyz - position);

        float spotEffect = dot(normalize(g_light_source[i].direction), normalize(-L));
        if (spotEffect > g_light_source[i].spot_cos_cutoff) {
        spotEffect = pow(spotEffect, g_light_source[i].spot_exponent);
        att = spotEffect / (g_light_source[i].constant_attenuation +
        g_light_source[i].linear_attenuation * dist +
        g_light_source[i].quadratic_attenuation * dist * dist);

        }
        else att = 0.0 ;
        }
        else if ( g_light_source[i].light_type == POINT_LIGHT ) {
        float dist = length(g_light_source[i].position.xyz - position);
        L = normalize(g_light_source[i].position.xyz - position);

        att = 1.0 / (g_light_source[i].constant_attenuation +
        g_light_source[i].linear_attenuation * dist +
        g_light_source[i].quadratic_attenuation * dist * dist);

        }

        vec3 E = normalize(-position); // we are in Eye Coordinates, so EyePos is (0,0,0)
        vec3 R = normalize(-reflect(L,N));

        //calculate Diffuse Term:

        vec4 Idiff = vec4(g_light_source[i].color, 1.0) * dc * max(dot(N,L), 0.0f);
        Idiff = clamp(Idiff, 0.0, 1.0);

        // calculate Specular Term:
        vec4 Ispec = vec4(g_light_source[i].color, 1.0) * g_material.specular
        * pow(max(dot(R,E),0.0f),g_material.shininess);
        Ispec = clamp(Ispec, 0.0, 1.0);

        finalColor +=  att*clamp(Ispec + Idiff, 0.0, 1.0);

        }

        return finalColor ;
        }
        )";

static string phong_fragment_shader_material = R"(
                                               void main (void)
                                               {
                                               FragColor = phongIllumination(g_material.diffuse);
                                               }
                                               )";

static string phong_fragment_shader_map = R"(
                                          in vec2 uv ;
                                          uniform sampler2D tex_unit;

                                          void main (void)
                                          {
                                          FragColor = phongIllumination(texture(tex_unit, uv));
                                          }
                                          )";

static string constant_fragment_shader = R"(
                                         #version 330

                                         uniform vec4 g_material_clr ;
                                         out vec4 FragColor;

                                         void main (void)
                                         {
                                         FragColor = g_material_clr ;
                                         }
                                         )";

static string pervertex_fragment_shader = R"(
                                          #version 330

                                          in vec3 color ;
                                          out vec4 FragColor;
                                          uniform float opacity ;

                                          void main (void)
                                          {
                                          FragColor = vec4(color, opacity) ;
                                          }
                                          )";

namespace cvx { namespace viz {

using MaterialPtr = std::shared_ptr<Material> ;

class PhongMaterial: public Material {
public:
    OpenGLShaderProgram::Ptr prog() override ;

    static MaterialPtr instance() {
        static MaterialPtr s_material(new PhongMaterial) ;
        return s_material ;
    }

    OpenGLShaderProgram::Ptr prog_ ;
};

OpenGLShaderProgram::Ptr PhongMaterial::prog()
{
    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, pn_vertex_shader_code, "pn_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, phong_fragment_shader_common + phong_fragment_shader_material, "phong_fragment_shader_material"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

void PhongMaterialInstance::applyParameters() {

    auto p = material_->prog() ;
    auto params = std::static_pointer_cast<PhongMaterialParameters>(params_) ;

    p->setUniform("g_material.ambient", params->ambient_) ;
    p->setUniform("g_material.diffuse", params->diffuse_) ;
    p->setUniform("g_material.specular", params->specular_) ;
    p->setUniform("g_material.shininess", params->shininess_) ;

}

PhongMaterialInstance::PhongMaterialInstance():
    MaterialInstance(PhongMaterial::instance(), make_shared<PhongMaterialParameters>()) {}

PhongMaterialInstance::PhongMaterialInstance(const std::shared_ptr<PhongMaterialParameters> &params):
    MaterialInstance(PhongMaterial::instance(),params) {}


class DiffuseMapMaterial: public Material {
public:
    OpenGLShaderProgram::Ptr prog() override ;

    static MaterialPtr instance() {
        static MaterialPtr s_material(new DiffuseMapMaterial) ;
        return s_material ;
    }

    OpenGLShaderProgram::Ptr prog_ ;
};

OpenGLShaderProgram::Ptr DiffuseMapMaterial::prog()
{
    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, pnt_vertex_shader_code, "pnt_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, phong_fragment_shader_common + phong_fragment_shader_map, "phong_fragment_shader_map"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

void DiffuseMapMaterialInstance::applyParameters() {

    auto p = material_->prog() ;
    auto params = std::static_pointer_cast<DiffuseMapMaterialParameters>(params_) ;

    p->setUniform("g_material.ambient", params->ambient_) ;
    p->setUniform("g_material.specular", params->specular_) ;
    p->setUniform("g_material.shininess", params->shininess_) ;

    p->setUniform("tex_unit", 0) ;

    params->diffuse_map_.read() ;
    params->diffuse_map_.upload() ;

}

DiffuseMapMaterialInstance::DiffuseMapMaterialInstance(const Texture2D &tex):
    MaterialInstance(DiffuseMapMaterial::instance(), std::make_shared<DiffuseMapMaterialParameters>(tex)){
}

DiffuseMapMaterialInstance::DiffuseMapMaterialInstance(const std::shared_ptr<DiffuseMapMaterialParameters> &params):
    MaterialInstance(DiffuseMapMaterial::instance(), params) {
}


class ConstantMaterial: public Material {
public:
    OpenGLShaderProgram::Ptr prog() override ;

    static MaterialPtr instance() {
        static MaterialPtr s_material(new ConstantMaterial) ;
        return s_material ;
    }

    OpenGLShaderProgram::Ptr prog_ ;
};

OpenGLShaderProgram::Ptr ConstantMaterial::prog() {

    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, p_vertex_shader_code, "p_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, constant_fragment_shader, "constant_fragment_shader"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

ConstantMaterialParameters::ConstantMaterialParameters(const Vector4f &clr): clr_(clr) {
}

ConstantMaterialInstance::ConstantMaterialInstance(const Vector4f &clr):
    MaterialInstance(ConstantMaterial::instance(), std::make_shared<ConstantMaterialParameters>(clr)){
}

ConstantMaterialInstance::ConstantMaterialInstance(const std::shared_ptr<ConstantMaterialParameters> &params):
    MaterialInstance(ConstantMaterial::instance(), params) {
}

void ConstantMaterialInstance::applyParameters() {
    auto p = material_->prog() ;
    auto params = std::static_pointer_cast<ConstantMaterialParameters>(params_) ;

    p->setUniform("g_material_clr", params->clr_) ;
}

class PerVertexColorMaterial: public Material {
public:
    OpenGLShaderProgram::Ptr prog() override ;

    static MaterialPtr instance() {
        static MaterialPtr s_material(new PerVertexColorMaterial) ;
        return s_material ;
    }

    OpenGLShaderProgram::Ptr prog_ ;
};

OpenGLShaderProgram::Ptr PerVertexColorMaterial::prog()
{
    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, pc_vertex_shader_code, "pc_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, pervertex_fragment_shader, "pervertex_fragment_shader"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

PerVertexColorMaterialInstance::PerVertexColorMaterialInstance(float op):
    MaterialInstance(PerVertexColorMaterial::instance(), std::make_shared<PerVertexColorMaterialParameters>(op)){
}

PerVertexColorMaterialInstance::PerVertexColorMaterialInstance(const std::shared_ptr<PerVertexColorMaterialParameters> &params):
    MaterialInstance(PerVertexColorMaterial::instance(), params) {
}

void PerVertexColorMaterialInstance::applyParameters() {
    auto p = material_->prog() ;
    auto params = std::static_pointer_cast<PerVertexColorMaterialParameters>(params_) ;

    p->setUniform("opacity", params->opacity_) ;
}


MaterialInstancePtr Material::makeLambertian(const Eigen::Vector4f &clr) {
    PhongMaterialInstance *p = new PhongMaterialInstance ;

    p->params().setDiffuse(clr) ;
    p->params().setSpecular(Vector4f(0, 0, 0, 1)) ;
    p->params().setShininess(0.0) ;
    return MaterialInstancePtr(p) ;
}

MaterialInstancePtr Material::makeConstant(const Eigen::Vector4f &clr) {
    return  MaterialInstancePtr(new ConstantMaterialInstance(clr)) ;
}

void MaterialInstance::use() {
    material_->prog()->use() ;
}

void MaterialInstance::applyDefaultPerspective(const Matrix4f &cam, const Matrix4f &view, const Matrix4f &model)
{
    Matrix4f mvp =  cam * view * model;
    Matrix4f mv =   view * model;

    Matrix3f wpi = mv.block<3, 3>(0, 0).transpose().eval() ;
    Matrix3f wp(wpi.inverse().eval()) ;

    auto p = material_->prog() ;

    p->setUniform("mvp", mvp) ;
    p->setUniform("mv", mv) ;
    p->setUniform("mvn", wp) ;
}

void MaterialInstance::applyDefaultLight(uint light_index, const LightPtr &light, const Isometry3f &tf)
{
    auto prog = material_->prog() ;

    string vname = cvx::util::format("g_light_source[%d]", light_index ++) ;

    if ( const auto &alight = std::dynamic_pointer_cast<AmbientLight>(light) ) {

        prog->setUniform(vname + ".light_type", 0) ;
        prog->setUniform(vname + ".color", alight->color_) ;
    }
    else if ( const auto &dlight = std::dynamic_pointer_cast<DirectionalLight>(light) ) {
        prog->setUniform(vname + ".light_type", 1) ;
        prog->setUniform(vname + ".color", dlight->diffuse_color_) ;
        prog->setUniform(vname + ".direction", tf * dlight->direction_) ;
    }
    else if ( const auto &slight = std::dynamic_pointer_cast<SpotLight>(light) ) {

        prog->setUniform(vname + ".light_type", 2) ;
        prog->setUniform(vname + ".color", slight->diffuse_color_) ;
        prog->setUniform(vname + ".direction", tf * slight->direction_) ;
        prog->setUniform(vname + ".position", tf * slight->position_) ;
        prog->setUniform(vname + ".constant_attenuation", slight->constant_attenuation_) ;
        prog->setUniform(vname + ".linear_attenuation", slight->linear_attenuation_) ;
        prog->setUniform(vname + ".quadratic_attenuation", slight->quadratic_attenuation_) ;
        prog->setUniform(vname + ".spot_exponent", slight->falloff_exponent_) ;
        prog->setUniform(vname + ".spot_cos_cutoff", (float)cos(M_PI*slight->falloff_angle_/180.0)) ;
    }
    else if ( const auto &plight = std::dynamic_pointer_cast<PointLight>(light)) {
        prog->setUniform(vname + ".light_type", 3) ;
        prog->setUniform(vname + ".color", plight->diffuse_color_) ;
        prog->setUniform(vname + ".position", tf * plight->position_) ;
        prog->setUniform(vname + ".constant_attenuation", plight->constant_attenuation_) ;
        prog->setUniform(vname + ".linear_attenuation", plight->linear_attenuation_) ;
        prog->setUniform(vname + ".quadratic_attenuation", plight->quadratic_attenuation_) ;
    }

}

void Texture2D::read()
{
    if ( im_.data ) return ;

    cv::Mat image ;
    string ipath ;

    if ( startsWith(image_url_, "file://" ) ) {
        ipath = image_url_.substr(7) ;
        im_ = cv::imread(ipath) ;
        cv::flip(im_, im_, 0) ;
    }
}

void Texture2D::upload()
{
    if ( !im_.data ) return ;

    if ( texture_id_ == 0 ) {
        glActiveTexture(GL_TEXTURE0);
        glGenTextures(1, &texture_id_);
        glBindTexture(GL_TEXTURE_2D, texture_id_);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Set texture clamping method
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
        glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
        glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0 );
        glPixelStorei( GL_UNPACK_SKIP_ROWS, 0 );

        glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                     0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGB,            // Internal colour format to convert to
                     im_.cols,
                     im_.rows,
                     0,                 // Border width in pixels (can either be 1 or 0)
                     GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE,  // Image data type
                     im_.ptr());        // The actual image data itself

        glGenerateMipmap(GL_TEXTURE_2D);
    } else {
        glActiveTexture(GL_TEXTURE0) ;
        glBindTexture(GL_TEXTURE_2D, texture_id_) ;
    }

}



}}
