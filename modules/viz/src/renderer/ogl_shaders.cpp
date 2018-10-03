#include <cvx/viz/renderer/ogl_shaders.hpp>
#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/filesystem.hpp>
#include <cvx/util/misc/path.hpp>

#include <cstring>
#include <fstream>

using namespace std ;
using namespace Eigen ;
using namespace cvx::util ;

OpenGLShader::OpenGLShader(Type t, const std::string &code, const string &rname): type_(t) {
    compileString(code, rname) ;
}

void OpenGLShader::compileString(const std::string &code, const string &resource_name) {
    if ( ( handle_ = glCreateShader(GLenum(type_)) ) == 0 )
        throw OpenGLShaderError("cannot create shader") ;

    const GLchar* p[1];
    p[0] = code.c_str() ;
    GLint lengths[1] = { (GLint)code.length() };

    glShaderSource(handle_, 1, p, lengths);
    glCompileShader(handle_);

    GLint success;
    glGetShaderiv(handle_, GL_COMPILE_STATUS, &success);

    if ( !success ) {
        GLchar info_log[1024];
        glGetShaderInfoLog(handle_, 1024, NULL, info_log);

        if ( resource_name.empty()) throw OpenGLShaderError(format("Error compilining shader: %s", info_log)) ;
        else throw OpenGLShaderError(format("Error compilining shader (%s): %s", resource_name, info_log)) ;
    }
}

void OpenGLShader::compileFile(const std::string &fname, const string &resource_name) {

    string contents = get_file_contents(fname) ;

    if ( contents.empty() ) {
        if ( resource_name.empty()) throw OpenGLShaderError(format("Error reading shader file: %s", fname)) ;
        else throw OpenGLShaderError(format("Error reading shader file (%s): %s", resource_name, fname)) ;
    }

    if ( resource_name.empty() )
        compileString(contents, Path(fname).name()) ;
    else
        compileString(contents, resource_name) ;
}

OpenGLShader::~OpenGLShader() {
    glDeleteShader(handle_) ;
}

OpenGLShaderProgram::OpenGLShaderProgram(const char *vshader_code, const char *fshader_code)
{
    handle_ = glCreateProgram();

    OpenGLShader::Ptr vertex_shader = make_shared<OpenGLShader>(OpenGLShader::Vertex, vshader_code) ;
    OpenGLShader::Ptr fragment_shader = make_shared<OpenGLShader>(OpenGLShader::Fragment, fshader_code) ;

    shaders_.push_back(vertex_shader) ;
    shaders_.push_back(fragment_shader) ;

    link();
}

void OpenGLShaderProgram::setUniform(const string &name, float v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform1f(loc, v) ;
}

void OpenGLShaderProgram::setUniform(const string &name, GLuint v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform1ui(loc, v) ;
}

void OpenGLShaderProgram::setUniform(const string &name, GLint v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform1i(loc, v) ;
}

void OpenGLShaderProgram::setUniform(const string &name, const Vector3f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform3fv(loc, 1, v.data()) ;
}

void OpenGLShaderProgram::setUniform(const string &name, const Vector4f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform4fv(loc, 1, v.data()) ;
}

void OpenGLShaderProgram::setUniform(const string &name, const Matrix3f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniformMatrix3fv(loc, 1, GL_FALSE, v.data()) ;
}

void OpenGLShaderProgram::setUniform(const string &name, const Matrix4f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniformMatrix4fv(loc, 1, GL_FALSE, v.data()) ;
}

OpenGLShaderProgram::~OpenGLShaderProgram() {
    glDeleteProgram(handle_) ;
}

OpenGLShaderProgram::OpenGLShaderProgram() {
    handle_ = glCreateProgram() ;
}

void OpenGLShaderProgram::addShader(const OpenGLShader::Ptr &shader) {
    shaders_.push_back(shader) ;
}

void OpenGLShaderProgram::addShaderFromString(OpenGLShader::Type t, const string &code, const string &resource_name) {
    auto shader = std::make_shared<OpenGLShader>(t) ;
    shader->compileString(code, resource_name) ;
    addShader(shader) ;
}

void OpenGLShaderProgram::addShaderFromFile(OpenGLShader::Type t, const string &fname, const string &resource_name) {
    auto shader = std::make_shared<OpenGLShader>(t) ;
    shader->compileFile(fname, resource_name) ;
    addShader(shader) ;
}


void OpenGLShaderProgram::link(bool validate) {


#if 0
    const GLchar* feedbackVaryings[] = { "gl_Position" };
    glTransformFeedbackVaryings(handle_, 1, feedbackVaryings, GL_INTERLEAVED_ATTRIBS);
#endif

    for( auto &&shader: shaders_ )
        glAttachShader(handle_, shader->handle()) ;

    GLchar error_log[1024] = { 0 };

    glLinkProgram(handle_) ;

    GLint success;
    glGetProgramiv(handle_, GL_LINK_STATUS, &success);

    if ( success == 0 ) {
        glGetProgramInfoLog(handle_, sizeof(error_log), NULL, error_log);
        throwError("Error linking shader program", error_log) ;
    }

    if ( validate ) {
        glValidateProgram(handle_);
        glGetProgramiv(handle_, GL_VALIDATE_STATUS, &success);

        if ( !success ) {
            glGetProgramInfoLog(handle_, sizeof(error_log), NULL, error_log);
            throwError("Invalid shader program", error_log);
        }
    }
}

void OpenGLShaderProgram::use() {
   glUseProgram(handle_) ;
}

void OpenGLShaderProgram::throwError(const char *error_str, const char *error_desc) {
    throw OpenGLShaderError(format("%: %", error_str, error_desc)) ;
}

OpenGLShader::Type type_from_string(const string &s) {
    if ( s == "vertex" ) return OpenGLShader::Vertex ;
    else if ( s == "fragment" ) return OpenGLShader::Fragment ;
    else if ( s == "geometry" ) return OpenGLShader::Geometry ;
    else if ( s == "compute" ) return OpenGLShader::Compute ;
    else if ( s == "tess_control" ) return OpenGLShader::TessControl ;
    else if ( s == "tess_evaluation" ) return OpenGLShader::TessEvaluation ;
    else return OpenGLShader::Fragment ;
}

OpenGLShader::Ptr OpenGLShaderLibrary::getOrCreateShader(const string &id, OpenGLShader::Type t, const string &code, const string &rname)
{
    auto it = shaders_.find(id) ;
    if ( it != shaders_.end() ) return it->second ;

    OpenGLShader::Ptr shader(new OpenGLShader(t)) ;
    shader->compileString(code, rname);
    shaders_.emplace(id, shader) ;
    return shader ;
}

void OpenGLShaderLibrary::addProgram(const string &id, OpenGLShaderProgram::Ptr p)
{
    p->link() ;
    programs_.emplace(id, p) ;
}

void OpenGLShaderLibrary::build(const vector<OpenGLShaderLibrary::ShaderConfig> &shaders, const vector<OpenGLShaderLibrary::ProgramConfig> &programs)
{
    for ( const auto &shader: shaders ) {
        auto p = std::make_shared<OpenGLShader>(type_from_string(shader.type_)) ;
        p->compileString(shader.src_, shader.path_) ;
        shaders_.emplace(shader.id_, p) ;
    }

    for ( const auto &prog: programs ) {
        auto p = std::make_shared<OpenGLShaderProgram>() ;
        for( const string &shader: prog.shaders_) {
            auto it = shaders_.find(shader) ;
            if ( it != shaders_.end() )
                p->addShader(it->second) ;
        }
        p->link() ;
        programs_.emplace(prog.id_, p) ;
    }
}

OpenGLShaderProgram::Ptr OpenGLShaderLibrary::get(const string &prog_name) {
    auto it = programs_.find(prog_name) ;
    if ( it != programs_.end() ) return it->second ;
    else return nullptr ;
}


OpenGLShaderError::OpenGLShaderError(const string &msg):
    std::runtime_error(msg) {}
