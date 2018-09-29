#include <cvx/viz/scene/material.hpp>

using namespace std ;
using namespace Eigen ;

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

OpenGLShaderProgram::Ptr PhongMaterial::prog() const
{
    static OpenGLShaderProgram::Ptr prog_ ;

    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, pn_vertex_shader_code, "pn_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, phong_fragment_shader_common + phong_fragment_shader_material, "phong_fragment_shader_material"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

void PhongMaterial::apply() {

    auto p = prog() ;

    p->setUniform("g_material.ambient", ambient_) ;
    p->setUniform("g_material.diffuse", diffuse_) ;
    p->setUniform("g_material.specular", specular_) ;
    p->setUniform("g_material.shininess", shininess_) ;
}

OpenGLShaderProgram::Ptr DiffuseMapMaterial::prog() const
{
    static OpenGLShaderProgram::Ptr prog_ ;

    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, pnt_vertex_shader_code, "pnt_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, phong_fragment_shader_common + phong_fragment_shader_map, "phong_fragment_shader_map"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

void DiffuseMapMaterial::apply() {

    auto p = prog() ;

    p->setUniform("g_material.ambient", ambient_) ;
    p->setUniform("g_material.specular", specular_) ;
    p->setUniform("g_material.shininess", shininess_) ;

    p->setUniform("tex_unit", 0) ;
}


OpenGLShaderProgram::Ptr ConstantMaterial::prog() const
{
    static OpenGLShaderProgram::Ptr prog_ ;

    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, p_vertex_shader_code, "p_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, constant_fragment_shader, "constant_fragment_shader"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

void ConstantMaterial::apply() {
    auto p = prog() ;

    p->setUniform("g_material_clr", clr_) ;
}

OpenGLShaderProgram::Ptr PerVertexColorMaterial::prog() const
{
    static OpenGLShaderProgram::Ptr prog_ ;

    if ( prog_ ) return prog_ ;

    OpenGLShader::Ptr vs(new OpenGLShader(OpenGLShader::Vertex, pc_vertex_shader_code, "pc_vertex_shader_code")) ;
    OpenGLShader::Ptr fs(new OpenGLShader(OpenGLShader::Fragment, pervertex_fragment_shader, "pervertex_fragment_shader"))  ;

    prog_.reset(new OpenGLShaderProgram) ;
    prog_->addShader(vs) ;
    prog_->addShader(fs) ;

    prog_->link() ;

    return prog_ ;
}

void PerVertexColorMaterial::apply() {
    auto p = prog() ;

    p->setUniform("opacity", opacity_) ;
}


MaterialPtr Material::makeLambertian(const Eigen::Vector4f &clr) {
    PhongMaterial *p = new PhongMaterial ;
    p->setDiffuse(clr) ;
    p->setSpecular(Vector4f(0, 0, 0, 1)) ;
    p->setShininess(0.0) ;
    return MaterialPtr(p) ;
}

MaterialPtr Material::makeConstant(const Eigen::Vector4f &clr) {
    return  MaterialPtr(new ConstantMaterial(clr)) ;
}


}}
