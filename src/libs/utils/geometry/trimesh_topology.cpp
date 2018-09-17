#include <cvx/util/geometry/trimesh_topology.hpp>

#include <memory>
#include <cassert>
#include <iostream>

using namespace std ;

namespace cvx { namespace util {

TriangleMeshTopology::TriangleMeshTopology(const TriangleMeshTopology &other)
{
    for( uint i=0 ; i<other.getNumFaces() ; i++ ) {
        vertex_idx_t v1, v2, v3 ;

        other.faceGetVertices(i, v1, v2, v3) ;

        addFace(v1, v2, v3) ;
    }
}

TriangleMeshTopology::TriangleMeshTopology(const std::vector<uint32_t> &triangles)
{
    for( uint i=0 ; i<triangles.size() ; ) {
        addFace(triangles[i++], triangles[i++], triangles[i++]) ;
    }

}

TriangleMeshTopology::TriangleMeshTopology() {
    n_edges_ = n_faces_ = 0 ;
    c_face_ = 0 ;
}

TriangleMeshTopology::~TriangleMeshTopology() {

}

TriangleMeshTopology::HalfEdge *TriangleMeshTopology::createHalfEdge(vertex_idx_t v1, vertex_idx_t v2, face_idx_t face)
{
    HalfEdge *edge = alloc_.allocate(1) ;

    edge->mate_ = NULL ;
    edge->face_ = face ;
    edge->next_ = 0 ;
    edge->prev_ = 0 ;

    HalfEdgeList::iterator it1 = emap_.find(EdgeKey(v1, v2)) ;
    HalfEdgeList::iterator it2 = emap_.find(EdgeKey(v2, v1)) ;

    if ( it1 != emap_.end() && it2 != emap_.end() ) // error - non manifold or duplicate
    {
        return NULL ;
    }
    else if ( it1 != emap_.end() ) // we found a half edge v1->v2 - error ccw direction ?
    {
        return NULL ;
    }
    else if ( it2 != emap_.end() ) // we found a half edge v2->v1
    {
        edge->vertex_ = v1 ;
        edge->mate_ = (*it2).second ;
        (*it2).second->mate_ = edge ;
        emap_[EdgeKey(v1, v2)] = edge ;
    }
    else // none found
    {
        edge->vertex_ = v1 ;
        edge->mate_ = NULL ;
        emap_[EdgeKey(v1, v2)] = edge ;
        vertex_map_[v1] = edge ;
        edge_map_[n_edges_++] = edge ;
    }

    return edge ;

}

face_idx_t TriangleMeshTopology::addFace(vertex_idx_t v1, vertex_idx_t v2, vertex_idx_t v3)
{
    bool cw = true ;
    HalfEdge *e1 = createHalfEdge(v1, v2, c_face_) ;
    HalfEdge *e2 = createHalfEdge(v2, v3, c_face_) ;
    HalfEdge *e3 = createHalfEdge(v3, v1, c_face_) ;

    if ( !e1 || !e2 || !e3 ) return -1 ;

    e1->next_ = e2 ;	e2->prev_ = e1 ;
    e2->next_ = e3 ; e3->prev_ = e2 ;
    e3->next_ = e1 ; e1->prev_ = e3 ;

    face_map_[c_face_] = e1 ;

    int ret = c_face_ ;

    c_face_ ++ ;
    n_faces_ ++ ;

    return ret ;
}


void TriangleMeshTopology::vtxGetNeighborVertices(vertex_idx_t vid, vector<vertex_idx_t> &vtcs) const
{
    HalfEdge *sEdge = (*vertex_map_.find(vid)).second ;

    HalfEdge *pEdge = sEdge, *qEdge ;

    vtcs.push_back(pEdge->next_->vertex_) ;

    do
    {
        pEdge = pEdge->next_->next_ ;

        if ( pEdge->mate_ != sEdge ) vtcs.push_back(pEdge->vertex_) ;

        pEdge = pEdge->mate_ ;
    } while ( pEdge && pEdge != sEdge ) ;

    if ( pEdge == sEdge ) return ;

    qEdge = sEdge->mate_ ;

    while ( qEdge && qEdge != sEdge->prev_)
    {
        qEdge = qEdge->next_->next_ ;

        vtcs.push_back(qEdge->next_->vertex_) ;

        qEdge = qEdge->mate_ ;
    }

}


bool TriangleMeshTopology::isBoundaryVertex(vertex_idx_t vtx) const
{
    HalfEdge *sEdge = (*vertex_map_.find(vtx)).second ;

    HalfEdge *pEdge = sEdge, *qEdge ;

    do  {
        if ( pEdge->mate_ == NULL ) return true ;

        pEdge = pEdge->next_->next_ ;

        pEdge = pEdge->mate_ ;
    } while ( pEdge && pEdge != sEdge ) ;

    if ( pEdge == sEdge ) return false ;

    qEdge = sEdge->mate_ ;

    while ( qEdge && qEdge != sEdge->prev_)
    {
        qEdge = qEdge->next_->next_ ;

        if ( qEdge->mate_ == NULL ) return true ;

        qEdge = qEdge->mate_ ;
    }

    return false ;
}

void TriangleMeshTopology::vtxGetNeighborFaces(vertex_idx_t vid, vector<face_idx_t> &faces) const
{
    HalfEdge *sEdge = (*vertex_map_.find(vid)).second;

    HalfEdge *pEdge = sEdge, *qEdge ;

    do
    {
        pEdge = pEdge->next_->next_ ;

        faces.push_back(pEdge->face_) ;

        pEdge = pEdge->mate_ ;
    } while ( pEdge && pEdge != sEdge ) ;


    if ( pEdge == sEdge ) return ;

    qEdge = sEdge->mate_ ;

    while ( qEdge && qEdge != sEdge->prev_)
    {
        qEdge = qEdge->next_->next_ ;

        faces.push_back(qEdge->next_->face_) ;

        qEdge = qEdge->mate_ ;
    }

}

void TriangleMeshTopology::edgeGetAdjFaces(vertex_idx_t v1, vertex_idx_t v2, face_idx_t &f1, face_idx_t &f2) const
{
    HalfEdge *pEdge = findEdge(v1, v2) ;

    if ( !pEdge ) return ;

    f1 = pEdge->face_ ;
    f2 = ( pEdge->mate_ ) ? pEdge->mate_->face_ : -1 ;

    if ( pEdge->vertex_ == v2 ) swap(f1, f2) ;
}

void TriangleMeshTopology::edgeGetAdjVertices(vertex_idx_t v1, vertex_idx_t v2, face_idx_t &f1, face_idx_t &f2) const
{
    HalfEdge *pEdge = findEdge(v1, v2) ;

    if ( !pEdge ) return ;

    f1 = pEdge->prev_->vertex_ ;
    f2 = (pEdge->mate_) ? pEdge->mate_->prev_->vertex_ : -1 ;

    if ( pEdge->vertex_ == v2 ) swap(f1, f2) ;
}

void TriangleMeshTopology::faceGetVertices(face_idx_t fid, std::vector<vertex_idx_t> &coordIndex) const
{
    int v1, v2, v3 ;

    faceGetVertices(fid, v1, v2, v3) ;

    coordIndex.push_back(v1) ;
    coordIndex.push_back(v2) ;
    coordIndex.push_back(v3) ;
}

void TriangleMeshTopology::faceGetVertices(face_idx_t fid, vertex_idx_t &v1, vertex_idx_t &v2, vertex_idx_t &v3) const
{
    map<int, HalfEdge *>::const_iterator it = face_map_.find(fid) ;

    if ( it == face_map_.end() )  {
        v1 = v2 = v3 = -1 ;
        return ;
    }

    HalfEdge *p = (*it).second ;

    v1 = p->vertex_ ;
    v2 = p->next_->vertex_ ;
    v3 = p->next_->next_->vertex_ ;
}


TriangleMeshTopology::HalfEdge *TriangleMeshTopology::findHalfEdge(int v1, int v2) const
{
    HalfEdgeList::const_iterator it = emap_.find(EdgeKey(v1, v2)) ;

    if ( it == emap_.end() ) return NULL ;
    else return (*it).second ;
}

void TriangleMeshTopology::fixVertexMap(HalfEdge *e1)
{
    HalfEdge *sEdge = e1, *pEdge = sEdge, *qEdge ;
    int v1 = e1->vertex_ ;

    bool found = false ;

    do
    {
        if ( pEdge != e1 ) {
            vertex_map_[v1] = pEdge ;
            found = true ;
            break ;
        }
        pEdge = pEdge->next_->next_->mate_ ;
    }  while ( pEdge && pEdge != sEdge ) ;

    qEdge = sEdge->mate_ ;

    while ( !found && qEdge && qEdge != sEdge->prev_)
    {
        qEdge = qEdge->next_->next_ ;

        if ( qEdge != e1 ) {
            found = true ;
            vertex_map_[v1] = qEdge ;
            break ;
        }

        qEdge = qEdge->mate_ ;
    }

    if ( !found ) vertex_map_.erase(v1) ;
}

void TriangleMeshTopology::collapseEdge(vertex_idx_t v1, vertex_idx_t v2)
{
    HalfEdge *e = findEdge(v1, v2), *e1, *e2 ;

    if ( !e ) return ;

    if ( e->vertex_ == v1 ) {
        e1 = e ;
        e2 = e->mate_ ;
    }
    else
    {
        e2 = e ;
        e1 = e->mate_ ;
    }

    // delete first face

    if ( e1 )
    {
        HalfEdge *en = e1->next_, *ep = e1->prev_ ;

        if ( ep->mate_ ) { ep->mate_->mate_ = en->mate_ ; }
        if ( en->mate_ ) { en->mate_->mate_ = ep->mate_ ; }
        face_map_.erase(e1->face_) ;
        n_faces_ -- ;
    }

    // delete second face

    if ( e2 )
    {
        HalfEdge *en = e2->next_, *ep = e2->prev_ ;

        if ( ep->mate_ ) { ep->mate_->mate_ = en->mate_ ; }
        if ( en->mate_ ) { en->mate_->mate_ = ep->mate_ ; }
        face_map_.erase(e2->face_) ;
        n_faces_ -- ;
    }

    // v2 will be deleted so fix references to v2

    if ( e2 && e2->prev_->mate_ ) e2->prev_->mate_->vertex_ = v1 ;
    if ( e1 && e1->next_->mate_ ) e1->next_->mate_->next_->vertex_ = v1 ;

    // Fix vertex_map_ references

    vertex_map_.erase(v2) ;

    // since e1 will be deleted we should make vertex map show to a different vertex
    if ( vertex_map_[v1]== e1 ) fixVertexMap(e1) ;

    if ( e1 ) // delete halfedges in one face
    {
        int v3 = e1->prev_->vertex_ ;

        if ( vertex_map_[v3] == e1->prev_ ) fixVertexMap(e1->prev_) ;

        HalfEdge *en = e1->next_, *ep = e1->prev_ ;
        deleteHalfEdge(e1) ;
        deleteHalfEdge(en) ;
        deleteHalfEdge(ep) ;
    }


    if ( e2 ) // delete half edges in another face
    {
        int v3 = e2->prev_->vertex_ ;

        if ( vertex_map_[v3] == e2->prev_ ) fixVertexMap(e2->prev_) ;

        HalfEdge *en = e2->next_, *ep = e2->prev_ ;
        deleteHalfEdge(e2) ;
        deleteHalfEdge(en) ;
        deleteHalfEdge(ep) ;
    }
}

void TriangleMeshTopology::deleteHalfEdge(HalfEdge *edge)
{
    EdgeKey key(edge->vertex_, edge->next_->vertex_) ;
    emap_.erase(key) ;
    alloc_.deallocate(edge, 1) ;
}

TriangleMeshTopology::HalfEdge *TriangleMeshTopology::findEdge(vertex_idx_t i1, vertex_idx_t i2) const
{
    HalfEdgeList::const_iterator it1 = emap_.find(EdgeKey(i1, i2)) ;
    HalfEdgeList::const_iterator it2 = emap_.find(EdgeKey(i2, i1)) ;

    if ( it1 == emap_.end() && it2 == emap_.end()) return NULL ;
    else if ( it1 == emap_.end() ) return (*it2).second ;
    else return (*it1).second ;
}

void TriangleMeshTopology::deleteFace(face_idx_t fid)
{
    std::map<int, HalfEdge *>::const_iterator it = face_map_.find(fid) ;

    if ( it == face_map_.end() ) return ;

    HalfEdge *e = (*it).second ;
    int v ;

    v = e->vertex_ ;
    if ( vertex_map_[v] == e ) fixVertexMap(e) ;
    e = e->next_ ;
    v = e->vertex_ ;
    if ( vertex_map_[v] == e ) fixVertexMap(e) ;
    e = e->next_ ;
    v = e->vertex_ ;
    if ( vertex_map_[v] == e ) fixVertexMap(e) ;


    deleteHalfEdge(e) ;
    deleteHalfEdge(e->next_) ;
    deleteHalfEdge(e->next_->next_) ;

    face_map_.erase(fid) ;
}

void TriangleMeshTopology::printTopology()
{
    map<int, HalfEdge *>::iterator itv = vertex_map_.begin() ;

    cout << "Vertices\n" ;
    while ( itv != vertex_map_.end() )
    {
        int vtx = (*itv).first ;
        cout << vtx << endl ;
        ++itv ;
    }


    map<int, HalfEdge *>::iterator it = face_map_.begin() ;

    cout << "Faces\n" ;
    while ( it != face_map_.end() )
    {
        face_idx_t face = (*it).second->face_ ;
        cout << face << ": " ;

        vertex_idx_t v1, v2, v3 ;
        faceGetVertices(face, v1, v2, v3) ;

        cout << v1 << ' ' << v2 << ' ' << v3  ;
        cout << endl ;

        ++it ;
    }
    cout << "Edges\n" ;

    int k = 0 ;
    map<int, HalfEdge *>::iterator ite = edge_map_.begin() ;

    while ( ite != edge_map_.end() )
    {
        HalfEdge *e = (*ite).second ;

        cout << k << ": " ;
        cout << e->vertex_ << ' ' ;
        cout << e->next_->vertex_ << endl ;

        ++ite ; ++k ;
    }

    cout << "Half Edges\n" ;

    k = 0 ;
    HalfEdgeList::iterator ithe = emap_.begin() ;

    while ( ithe != emap_.end() )
    {
        HalfEdge *e = (*ithe).second ;
        EdgeKey key = (*ithe).first ;

        cout << e << ": " ;
        cout << e->vertex_ << ' ' ;
        cout << e->next_->vertex_ << ' ' ;
        cout << e->face_ << ' ' ;
        cout << e->mate_ << endl ;

        ++ithe ; ++k ;
    }
}

void TriangleMeshTopology::getEdge(vertex_idx_t idx, vertex_idx_t &v1, vertex_idx_t &v2, face_idx_t &f1, face_idx_t &f2) const
{
    v1 = v2 = f1 = f2 = -1 ;

    map<int, HalfEdge *>::const_iterator it = edge_map_.find(idx) ;

    if ( it == edge_map_.end() ) return ;

    HalfEdge *p = (*it).second ;

    v1 = p->vertex_ ;
    v2 = p->next_->vertex_ ;
    f1 = p->face_ ;
    f2 = ( p->mate_ ) ? p->mate_->face_ : -1 ;
}


//////////////////////////////////////////////////////////////////////////

VertexVertexIterator::VertexVertexIterator(const TriangleMeshTopology &topo, vertex_idx_t vtx)
{
    // Find leftmost edge
    std::map<int, TriangleMeshTopology::HalfEdge *>::const_iterator it = topo.vertex_map_.find(vtx) ;

    assert(it != topo.vertex_map_.end() ) ;

    s_edge_ = (*it).second ;

    TriangleMeshTopology::HalfEdge *pEdge = s_edge_ ;

    do {
        if ( pEdge->mate_ ) pEdge = pEdge->mate_->next_ ;
        else break ;
    }
    while ( pEdge && pEdge != s_edge_ ) ;

    s_edge_ = c_edge_ = pEdge ;

}

VertexVertexIterator & VertexVertexIterator::operator = (const VertexVertexIterator &other)
{
    s_edge_ = other.s_edge_ ;
    c_edge_ = other.c_edge_ ;

    return *this ;
}

bool VertexVertexIterator::operator == (const VertexVertexIterator &other) const
{
    return ( s_edge_ == other.s_edge_ && c_edge_ == other.c_edge_ ) ;
}

bool VertexVertexIterator::operator != (const VertexVertexIterator &other) const
{
    return !(*this == other) ;
}

VertexVertexIterator & VertexVertexIterator::operator ++ ()
{
    if ( c_edge_ )
    {
        if ( c_edge_ == s_edge_ ) c_edge_ = c_edge_->next_ ;
        else if ( c_edge_->next_->mate_ ) {
            if ( c_edge_->next_->mate_->prev_->mate_ &&
                 c_edge_->next_->mate_->prev_->mate_ == s_edge_ ) c_edge_ = NULL ;
            else c_edge_ = c_edge_->next_->mate_->next_ ;
        }
        else c_edge_ = NULL ;
    }

    return *this ;
}


vertex_idx_t VertexVertexIterator::operator * () const
{
    return c_edge_->next_->vertex_ ;
}

VertexVertexIterator::operator bool () const
{
    return c_edge_ != 0 ;
}

//////////////////////////////////////////////////////////////////////////////////////


VertexFaceIterator::VertexFaceIterator(const TriangleMeshTopology &topo, int vtx)
{

    // Find leftmost edge
    std::map<int, TriangleMeshTopology::HalfEdge *>::const_iterator it = topo.vertex_map_.find(vtx) ;

    assert(it != topo.vertex_map_.end() ) ;

    s_edge_ = (*it).second ;

    TriangleMeshTopology::HalfEdge *pEdge = s_edge_, *qEdge ;

    do {
        if ( pEdge->mate_ ) pEdge = pEdge->mate_->next_ ;
        else break ;
    }
    while ( pEdge && pEdge != s_edge_ ) ;

    s_edge_ = c_edge_ = pEdge ;

}

VertexFaceIterator & VertexFaceIterator::operator = (const VertexFaceIterator &other)
{
    s_edge_ = other.s_edge_ ;
    c_edge_ = other.c_edge_ ;

    return *this ;
}

bool VertexFaceIterator::operator == (const VertexFaceIterator &other) const
{
    return ( s_edge_ == other.s_edge_ && c_edge_ == other.c_edge_ ) ;
}

bool VertexFaceIterator::operator != (const VertexFaceIterator &other) const
{
    return !(*this == other) ;
}

VertexFaceIterator & VertexFaceIterator::operator ++ ()
{
    if ( c_edge_ )
    {
        c_edge_ = c_edge_->next_->next_->mate_ ;
        if ( c_edge_ == s_edge_ ) c_edge_ = NULL ;
    }

    return *this ;

}


int VertexFaceIterator::operator * () const
{
    return c_edge_->face_ ;
}

VertexFaceIterator::operator bool () const
{
    return c_edge_ != 0 ;
}

/////////////////////////////////////////////////////////////////////////////////////////////

EdgeIterator::EdgeIterator(const TriangleMeshTopology &topo)
{
    it_ = topo.edge_map_.begin() ;
    last_ = topo.edge_map_.end() ;
}

EdgeIterator & EdgeIterator::operator = (const EdgeIterator &other)
{
    it_ = other.it_ ;
    return *this ;
}

bool EdgeIterator::operator == (const EdgeIterator &other) const
{
    return ( it_ == other.it_ ) ;
}

bool EdgeIterator::operator != (const EdgeIterator &other) const
{
    return !(*this == other) ;
}

EdgeIterator & EdgeIterator::operator ++ ()
{
    ++it_ ;
    return *this ;
}


std::pair<vertex_idx_t, vertex_idx_t> EdgeIterator::operator * () const
{
    TriangleMeshTopology::HalfEdge *he = (*it_).second ;
    return std::make_pair(he->vertex_, he->next_->vertex_) ;
}

EdgeIterator::operator bool () const
{
    return it_ != last_ ;
}

}}
