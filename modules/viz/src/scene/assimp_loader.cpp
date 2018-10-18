#include <cvx/viz/scene/scene.hpp>
#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/scene/drawable.hpp>
#include <cvx/viz/scene/mesh.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/material.hpp>

#include <cvx/util/misc/path.hpp>

#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include <Eigen/Dense>

#include <cvx/util/misc/optional.hpp>

using namespace std ;
using namespace Eigen ;
using cvx::util::optional ;
using cvx::util::Path ;

static Vector4f color4_to_float4(const aiColor4D &c) {
    return Vector4f(c.r, c.g, c.b, c.a) ;
}

namespace cvx { namespace viz { namespace impl {


class AssimpImporter {
public:
    AssimpImporter(Scene &sc, bool make_pickable): scene_(sc), make_pickable_(make_pickable) {}

    MaterialInstancePtr importMaterial(const struct aiMaterial *mtl, const string &model_path) ;

    Scene &scene_ ;

    map<const aiMesh *, MeshPtr> meshes_ ;
    map<const aiMaterial *, MaterialInstancePtr> materials_ ;
    map<string, LightPtr> lights_ ;
    map<string, CameraPtr> cameras_ ;
    bool make_pickable_ ;

    bool importMaterials(const string &mpath, const aiScene *sc);
    bool importMeshes(const aiScene *sc);
    bool importLights(const aiScene *sc);
    bool importNodes(NodePtr &pnode, const aiScene *sc, const aiNode *nd);
    bool import(const aiScene *sc, const std::string &fname, const NodePtr &parent);
};

static void getPhongMaterial(const struct aiMaterial *mtl,
                             optional<Vector4f> &vambient,
                             optional<Vector4f> &vdiffuse,
                             optional<Vector4f> &vspecular,
                             optional<float> &vshininess) {

    aiColor4D diffuse, specular, ambient;
    float shininess, strength;
    unsigned int max;

    if ( AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
        vdiffuse = color4_to_float4(diffuse);

    if ( AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular) )
        vspecular = color4_to_float4(specular) ;

    if ( AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient) )
        vambient = color4_to_float4(ambient) ;

    max = 1;
    aiReturn ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);

    if ( ret1 == AI_SUCCESS ) {
        max = 1;
        aiReturn ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
        if(ret2 == AI_SUCCESS) shininess = shininess * strength ;

        vshininess = shininess ;
    }
    else {
        vshininess = 0 ;
        vspecular = Vector4f(0, 0, 0, 1) ;
    }
}

static void getMaterialTexture(const struct aiMaterial *mtl, optional<Texture2D> &texture, const string &model_path) {
    Path mpath(model_path) ;

    aiString tex_path ;
    aiTextureMapping tmap ;
    aiTextureMapMode mode ;

    if ( AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, 0, &tex_path, &tmap, 0, 0, 0, &mode) ) {
        string file_name(tex_path.data, tex_path.length) ;

        string url = "file://" + (mpath.parentPath() / file_name).toString() ;
        Texture2D sampler(url) ;
        texture = sampler ;
    }
}


MaterialInstancePtr AssimpImporter::importMaterial(const struct aiMaterial *mtl, const string &model_path) {

    int shading_model ;
    mtl->Get((const char *)AI_MATKEY_SHADING_MODEL, shading_model);

    optional<Texture2D> diffuse_map ;
    optional<Vector4f> ambient, diffuse, specular ;
    optional<float> shininess ;

    getMaterialTexture(mtl, diffuse_map, model_path) ;
    getPhongMaterial(mtl, ambient, diffuse, specular, shininess) ;

    if ( diffuse_map ) {
        DiffuseMapMaterialInstance *mat = new DiffuseMapMaterialInstance(diffuse_map.value()) ;
        if ( ambient ) mat->params().setAmbient(ambient.value()) ;
        mat->params().setDiffuse(diffuse_map.value()) ;
        if ( specular ) mat->params().setSpecular(specular.value()) ;
        if ( shininess ) mat->params().setShininess(shininess.value()) ;
        return MaterialInstancePtr(mat) ;
    } else {
        PhongMaterialInstance *mat = new PhongMaterialInstance() ;
        if ( ambient ) mat->params().setAmbient(ambient.value()) ;
        if ( diffuse ) mat->params().setDiffuse(diffuse.value()) ;
        if ( specular ) mat->params().setSpecular(specular.value()) ;
        if ( shininess ) mat->params().setShininess(shininess.value()) ;
        return MaterialInstancePtr(mat) ;
    }

}

bool AssimpImporter::importMaterials(const string &mpath, const aiScene *sc) {
    for( uint m=0 ; m<sc->mNumMaterials ; m++ ) {
        const aiMaterial *material = sc->mMaterials[m] ;
        MaterialInstancePtr smat = importMaterial(material, mpath) ;
        materials_[material] = smat ;
    }

    return true ;
}

bool AssimpImporter::importMeshes(const aiScene *sc) {

    for( uint m=0 ; m<sc->mNumMeshes ; m++ ) {
        const aiMesh *mesh = sc->mMeshes[m] ;

        //   if ( mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE ) continue ;

        MeshPtr smesh ;

        if ( mesh->mPrimitiveTypes == aiPrimitiveType_TRIANGLE ) smesh.reset(new Mesh(Mesh::Triangles)) ;
        else if ( mesh->mPrimitiveTypes == aiPrimitiveType_LINE ) smesh.reset(new Mesh(Mesh::Lines)) ;
        else if ( mesh->mPrimitiveTypes == aiPrimitiveType_POINT ) smesh.reset(new Mesh(Mesh::Points)) ;
        else continue ;

        if ( mesh->HasPositions() ) {
            uint n = mesh->mNumVertices ;
            smesh->vertices().data().resize(n) ;

            for(int i = 0; i < n; ++i)
                smesh->vertices().data()[i] = Vector3f(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z) ;
        }

        if ( mesh->HasNormals() ) {
            uint n = mesh->mNumVertices ;
            smesh->normals().data().resize(n) ;

            for(int i = 0; i < n; ++i)
                smesh->normals().data()[i] = Vector3f(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z) ;
        }

        if ( mesh->HasVertexColors(0) ) {
            uint n = mesh->mNumVertices ;
            smesh->colors().data().resize(n) ;

            for(int i = 0; i < n; ++i)
                smesh->colors().data()[i] = Vector3f(mesh->mColors[0][i].r, mesh->mColors[0][i].g, mesh->mColors[0][i].b) ;
        }

        for( uint t=0 ; t<MAX_TEXTURES ; t++ ) {
            uint n = mesh->mNumVertices ;
            if ( mesh->HasTextureCoords(t) ) {
                smesh->texCoords(t).data().resize(n) ;

                for(int i = 0; i < n; ++i)
                    smesh->texCoords(t).data()[i] = Vector2f(mesh->mTextureCoords[t][i].x, mesh->mTextureCoords[t][i].y) ;
            }
        }

        if ( mesh->HasFaces() ) {
            uint n = mesh->mNumFaces ;
            smesh->vertices().indices().resize(n * 3) ;

            for(int i = 0, k=0; i < n ; i++) {
                smesh->vertices().indices()[k++] = mesh->mFaces[i].mIndices[0];
                smesh->vertices().indices()[k++] = mesh->mFaces[i].mIndices[1];
                smesh->vertices().indices()[k++] = mesh->mFaces[i].mIndices[2];
            }
        }

        if ( smesh->ptype() == Mesh::Triangles && make_pickable_ )
            smesh->makeOctree() ;

        meshes_[mesh] = smesh ;
    }

    return true ;
}

bool AssimpImporter::importLights(const aiScene *sc) {
    for( uint m=0 ; m<sc->mNumLights ; m++ ) {
        const aiLight *light = sc->mLights[m] ;

        LightPtr slight ;

        switch ( light->mType ) {
        case aiLightSource_DIRECTIONAL:
        {
            DirectionalLight *dl = new DirectionalLight({light->mDirection.x, light->mDirection.y, light->mDirection.z}) ;
            dl->diffuse_color_ << light->mColorDiffuse.r, light->mColorDiffuse.g, light->mColorDiffuse.b ;
            dl->specular_color_ << light->mColorSpecular.r, light->mColorSpecular.g, light->mColorSpecular.b ;
            dl->ambient_color_ << light->mColorAmbient.r, light->mColorAmbient.g, light->mColorAmbient.b ;

            slight.reset(dl) ;
            break ;
        }
        case aiLightSource_POINT:
        {
            PointLight *pl = new PointLight({light->mPosition.x, light->mPosition.y, light->mPosition.z}) ;

            pl->diffuse_color_ << light->mColorDiffuse.r, light->mColorDiffuse.g, light->mColorDiffuse.b ;
            pl->specular_color_ << light->mColorSpecular.r, light->mColorSpecular.g, light->mColorSpecular.b ;
            pl->ambient_color_ << light->mColorAmbient.r, light->mColorAmbient.g, light->mColorAmbient.b ;

            pl->constant_attenuation_ = light->mAttenuationConstant ;
            pl->linear_attenuation_ = light->mAttenuationLinear ;
            pl->quadratic_attenuation_ = light->mAttenuationQuadratic ;

            slight.reset(pl) ;
            break ;

        }
        case aiLightSource_SPOT:
        {
            SpotLight *sl = new SpotLight({light->mPosition.x, light->mPosition.y, light->mPosition.z},
                                    {light->mDirection.x, light->mDirection.y, light->mDirection.z}) ;

            sl->diffuse_color_ << light->mColorDiffuse.r, light->mColorDiffuse.g, light->mColorDiffuse.b ;
            sl->specular_color_ << light->mColorSpecular.r, light->mColorSpecular.g, light->mColorSpecular.b ;
            sl->ambient_color_ << light->mColorAmbient.r, light->mColorAmbient.g, light->mColorAmbient.b ;

            sl->constant_attenuation_ = light->mAttenuationConstant ;
            sl->linear_attenuation_ = light->mAttenuationLinear ;
            sl->quadratic_attenuation_ = light->mAttenuationQuadratic ;
            sl->falloff_angle_ = light->mAngleOuterCone ;
            sl->falloff_exponent_ = 0 ;

            slight.reset(sl) ;
            break ;

        }

        }

        if ( slight ) {
            slight->name_ = light->mName.C_Str() ;
            lights_[slight->name_] = slight ;
        }
    }



    return true ;
}

bool AssimpImporter::importNodes(NodePtr &pnode, const struct aiScene *sc, const struct aiNode* nd)
{
    unsigned int i;
    unsigned int n = 0, t;
    aiMatrix4x4 m = nd->mTransformation;
    /* update transform */

    NodePtr snode(new Node) ;


    Matrix4f tf ;
    tf << m.a1, m.a2, m.a3, m.a4,
            m.b1, m.b2, m.b3, m.b4,
            m.c1, m.c2, m.c3, m.c4,
            m.d1, m.d2, m.d3, m.d4 ;

    snode->matrix() = tf.eval() ;

    string nname(nd->mName.C_Str()) ;
    snode->setName(nname);

    /* draw all meshes assigned to this node */
    for (; n < nd->mNumMeshes; ++n) {

        const aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

        map<const aiMesh *, MeshPtr>::const_iterator mit = meshes_.find(mesh) ;

        if ( mit == meshes_.end() ) continue ;

        GeometryPtr geom  =  std::static_pointer_cast<Geometry>(mit->second) ;

        const aiMaterial* material = sc->mMaterials[mesh->mMaterialIndex];

        map<const aiMaterial *, MaterialInstancePtr>::const_iterator cit = materials_.find(material) ;

        MaterialInstancePtr mat ;

        if ( cit != materials_.end() )
            mat = cit->second ;

        DrawablePtr dr(new Drawable(geom, mat)) ;

        snode->addDrawable(dr) ;
    }

    auto lit = lights_.find(nname) ;
    if ( lit != lights_.end() )  // this is a light node
        snode->addLight(lit->second) ;

    auto cit = cameras_.find(nname) ;
    if ( cit != cameras_.end() )  ;// this is a camera node, no special handling

    if ( pnode ) pnode->addChild(snode) ;

    /* import all children */
    for (n = 0; n < nd->mNumChildren; ++n) {
        if ( !importNodes(snode, sc, nd->mChildren[n]) )
            return false ;
    }

    pnode->setPickable(make_pickable_) ;

    return true ;
}


bool AssimpImporter::import(const aiScene *sc, const std::string &fname, const NodePtr &parent) {

    if ( !importMeshes(sc) ) return false ;
    if ( !importMaterials(fname, sc) ) return false ;
    if ( !importLights(sc) ) return false ;

    NodePtr root = (parent) ? parent : scene_.shared_from_this() ;
    if ( !importNodes(root, sc, sc->mRootNode) ) return false ;

    return true ;
}

}

void Scene::load(const std::string &fname, const NodePtr &parent, bool make_pickable) {
  //  const aiScene *sc = aiImportFile(fname.c_str(), aiProcessPreset_TargetRealtime_MaxQuality | aiProcess_FlipUVs | aiProcess_TransformUVCoords);
    const aiScene *sc = aiImportFile(fname.c_str(),
    aiProcess_GenNormals
  | aiProcess_Triangulate
  | aiProcess_JoinIdenticalVertices
  | aiProcess_SortByPType
  | aiProcess_OptimizeMeshes
  |  aiProcess_FlipUVs | aiProcess_TransformUVCoords
                                     ) ;
    if ( !sc ) {
        throw SceneLoaderException(aiGetErrorString(), fname) ;
    }

    load(sc, fname, parent, make_pickable) ;

    aiReleaseImport(sc) ;
}

void Scene::load(const aiScene *sc, const std::string &fname, const NodePtr &parent, bool make_pickable) {

    impl::AssimpImporter importer(*this, make_pickable) ;

    bool res = importer.import(sc, fname, parent) ;

    if ( !res ) {
        aiReleaseImport(sc) ;
        throw SceneLoaderException("Error while parsing assimp scene", fname) ;
    }

}


}}

