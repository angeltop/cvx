#ifndef __TRIANGLE_MESH_TOPOLOGY_HPP__
#define __TRIANGLE_MESH_TOPOLOGY_HPP__

#include <map>
#include <vector>

namespace cvx { namespace util {

typedef int32_t face_idx_t ;
typedef int32_t vertex_idx_t ;

    class TriangleMeshTopology
    {
    public:

        TriangleMeshTopology() ;
        TriangleMeshTopology(const TriangleMeshTopology &other) ;
        TriangleMeshTopology(const std::vector<uint32_t> &triangles) ;


        ~TriangleMeshTopology() ;

        // Adds a face to the mesh. Takes as input the IDs of the vertices.
        // Returns the face ID. The order of vertices should always be clockwise

        face_idx_t addFace(vertex_idx_t v1, vertex_idx_t v2, vertex_idx_t v3) ;
        void deleteFace(face_idx_t faceID) ;

        void vtxGetNeighborVertices(vertex_idx_t vid, std::vector<vertex_idx_t> &vtcs) const ;
        void vtxGetNeighborFaces(vertex_idx_t vid, std::vector<face_idx_t> &faces) const ;

        // f1 is the face on the right of the oriented edge v1->v2
        void edgeGetAdjFaces(vertex_idx_t v1, vertex_idx_t v2, face_idx_t &f1, face_idx_t &f2) const ;
        // Get the vertices opposite and on the sides of the edge. a1 is on the right.
        void edgeGetAdjVertices(vertex_idx_t v1, vertex_idx_t v2, vertex_idx_t &a1, vertex_idx_t &a2) const ;

        void faceGetVertices(face_idx_t fid, vertex_idx_t &v1, vertex_idx_t &v2, vertex_idx_t &v3) const ;
        void faceGetVertices(face_idx_t fid, std::vector<vertex_idx_t> &coordIndex) const ;

        void collapseEdge(vertex_idx_t v1, vertex_idx_t v2) ;

        void printTopology() ;

        face_idx_t getNumFaces() const { return face_map_.size() ; }
        vertex_idx_t getNumVertices() const { return vertex_map_.size() ; }
        vertex_idx_t getNumEdges() const { return edge_map_.size() ; }

        void getEdge(vertex_idx_t idx, vertex_idx_t &v1, vertex_idx_t &v2, face_idx_t &f1, face_idx_t &f2) const;

        bool vtxExists(vertex_idx_t vid) const { return vertex_map_.count(vid) != 0 ; }


        // bool isBoundaryEdge(vertex_idx_t vtx) ;

    private:

        struct HalfEdge {
            HalfEdge() {}
            HalfEdge *mate_ ; // The twin half-edge at the opposite direction if not boundary edge.
            HalfEdge *prev_ ; // The previous half-edge in the face loop in cw direction
            HalfEdge *next_ ; // The next half-edge in the face loop in cw direction
            vertex_idx_t vertex_ ;	 // The ID of the vertex from which the half edge starts
            vertex_idx_t face_ ;		 // The ID of the face that this half-edge belongs to
        } ;

        HalfEdge *findEdge(vertex_idx_t v1, vertex_idx_t v2) const ;

        bool isBoundaryVertex(vertex_idx_t vtx) const ;
        bool isBoundaryFace(vertex_idx_t vtx) ;

        friend class VertexVertexIterator ;
        friend class VertexFaceIterator ;
        friend class EdgeIterator ;

        HalfEdge *findHalfEdge(vertex_idx_t v1, vertex_idx_t v2) const ;
        void fixVertexMap(HalfEdge *e1) ;
        void deleteHalfEdge(HalfEdge *) ;

        std::map<vertex_idx_t, HalfEdge *> vertex_map_ ; // list of vertices
        std::map<face_idx_t, HalfEdge *> face_map_ ;	// list of faces
        std::map<vertex_idx_t, HalfEdge *> edge_map_ ;	// list of edges

        typedef std::pair<vertex_idx_t, vertex_idx_t> EdgeKey ;
        typedef std::map<EdgeKey, HalfEdge *> HalfEdgeList ; // list of half edges indexed by source
        // and target vertices

        HalfEdgeList emap_ ;

        std::allocator<HalfEdge> alloc_ ;

        HalfEdge *createHalfEdge(vertex_idx_t v1, vertex_idx_t v2, vertex_idx_t face) ;

        HalfEdge *s_edge_, *e_edge_ ;
        face_idx_t c_face_ ;
        size_t n_edges_, n_faces_ ;

};

class VertexVertexIterator
{
public:

    VertexVertexIterator(const TriangleMeshTopology &topo, vertex_idx_t vtx) ;

    VertexVertexIterator &operator= (const VertexVertexIterator &other) ;

    bool operator == (const VertexVertexIterator &other) const ;
    bool operator != (const VertexVertexIterator &other) const ;
    VertexVertexIterator & operator ++ () ;

    vertex_idx_t operator * () const ;
    operator bool () const ;

private:

    TriangleMeshTopology::HalfEdge *s_edge_, *c_edge_ ;

} ;

class VertexFaceIterator
{
public:
     VertexFaceIterator(const TriangleMeshTopology &topo, vertex_idx_t vtx) ;

    VertexFaceIterator &operator= (const VertexFaceIterator &other) ;

    bool operator == (const VertexFaceIterator &other) const ;
    bool operator != (const VertexFaceIterator &other) const ;
    VertexFaceIterator & operator ++ () ;

    face_idx_t operator * () const ;
    operator bool () const ;

private:

    TriangleMeshTopology::HalfEdge *s_edge_, *c_edge_ ;

} ;

class EdgeIterator
{
public:
    EdgeIterator(const TriangleMeshTopology &topo) ;

    EdgeIterator &operator= (const EdgeIterator &other) ;

    bool operator == (const EdgeIterator &other) const ;
    bool operator != (const EdgeIterator &other) const ;
    EdgeIterator & operator ++ () ;

    std::pair<vertex_idx_t, vertex_idx_t> operator * () const ;
    operator bool () const ;

private:

    std::map<vertex_idx_t, TriangleMeshTopology::HalfEdge *>::const_iterator it_, last_ ;
};

}}

#endif
