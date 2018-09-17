#include <cvx/util/geometry/triangulate.hpp>

#include <iostream>
#include <fstream>
#include <map>

#include <string>
#include <vector>


using namespace std ;

extern "C" {
#define REAL float
#define VOID void
#define ANSI_DECLARATORS
#include <triangle.h>

}

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace util {

void triangulatePoints(const triangulation::pts_matrix_t &pts, const triangulation::attribute_matrix_t &attrs, double area,
                       triangulation::pts_matrix_t &out_pts, triangulation::attribute_matrix_t &out_attrs,
                       std::vector<uint32_t> &triangles)
{
    assert( pts.cols() == 2 ) ;
    assert( attrs.cols() > 0 ) ;
    assert( pts.rows() == attrs.rows() ) ;
    assert( area > 0 ) ;

    size_t n_pts = pts.rows() ;
    size_t n_attrs = attrs.cols() ;

    char triswitches[20] ;
    sprintf(triswitches, "zqQa%f", area) ;

    struct triangulateio trsin, trsout ;

    memset(&trsin, 0, sizeof(trsin)) ;
    memset(&trsout, 0, sizeof(trsin)) ;

    trsin.numberofpoints = n_pts ;
    trsin.pointlist = (REAL *)pts.data() ;

    trsin.numberofpointattributes = n_attrs ;
    trsin.pointattributelist = (REAL *)attrs.data() ;

    triangulate(triswitches, &trsin, &trsout, NULL) ;

    out_pts = Eigen::Map< triangulation::pts_matrix_t >(trsout.pointlist, trsout.numberofpoints, 2) ;

    for( size_t i=0, k=0 ; i<trsout.numberoftriangles ; i++ )
    {
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
    }

    out_attrs = Eigen::Map< triangulation::attribute_matrix_t >(trsout.pointattributelist, trsout.numberofpoints, n_attrs) ;

    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;
    free(trsout.pointattributelist) ;
}

void triangulatePoints(const triangulation::pts_matrix_t &pts, double area, triangulation::pts_matrix_t &out_pts, std::vector<uint32_t> &triangles) {

    assert( pts.cols() == 2 ) ;
    assert( area > 0 ) ;

    size_t n_pts = pts.rows() ;

    char triswitches[20] ;
    sprintf(triswitches, "zqQa%f", area) ;

    struct triangulateio trsin, trsout ;

    memset(&trsin, 0, sizeof(trsin)) ;
    memset(&trsout, 0, sizeof(trsin)) ;

    trsin.numberofpoints = n_pts ;
    trsin.pointlist = (REAL *)pts.data() ;

    triangulate(triswitches, &trsin, &trsout, NULL) ;

    out_pts = Eigen::Map< triangulation::pts_matrix_t >(trsout.pointlist, trsout.numberofpoints, 2) ;

    for( size_t i=0, k=0 ; i<trsout.numberoftriangles ; i++ )
    {
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
    }

    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;

}

void triangulatePoints(const triangulation::pts_matrix_t &pts, std::vector<uint32_t> &triangles) {

    assert( pts.cols() == 2 ) ;
    size_t n_pts = pts.rows() ;

    struct triangulateio trsin, trsout ;

    memset(&trsin, 0, sizeof(trsin)) ;
    memset(&trsout, 0, sizeof(trsin)) ;

    trsin.numberofpoints = n_pts ;
    trsin.pointlist = (REAL *)pts.data() ;

    triangulate((char *)"zQ", &trsin, &trsout, NULL) ;

    for( size_t i=0, k=0 ; i<trsout.numberoftriangles ; i++ )
    {
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
    }

    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;

}


static vector<int> make_poly_segments(size_t np) {
    vector<int> res ;

    for( uint i=0 ; i<np-1 ; i++) {
        res.push_back(i) ; res.push_back(i+1) ;
    }

    res.push_back(np-1) ;
    res.push_back(0) ;

    return res ;
}

void triangulateConstraint(const triangulation::pts_matrix_t &pts, const std::vector<int> &segments, const triangulation::attribute_matrix_t &attrs, double area,
                       triangulation::pts_matrix_t &out_pts, triangulation::attribute_matrix_t &out_attrs, std::vector<uint32_t> &triangles) {

    assert( pts.cols() == 2 ) ;
    assert( attrs.cols() > 0 ) ;
    assert( pts.rows() == attrs.rows() ) ;
    assert( area > 0 ) ;

    size_t n_segments = segments.size()/2 ;
    size_t n_pts = pts.rows() ;
    size_t n_attrs = attrs.cols() ;

    char triswitches[20] ;
    sprintf(triswitches, "zqpQa%f", area) ;

    struct triangulateio trsin, trsout ;

    memset(&trsin, 0, sizeof(trsin)) ;
    memset(&trsout, 0, sizeof(trsin)) ;

    trsin.numberofpoints = n_pts ;
    trsin.pointlist = (REAL *)pts.data() ;
    trsin.numberofpointattributes = n_attrs ;
    trsin.pointattributelist = (REAL *)attrs.data() ;

    trsin.numberofsegments = n_segments ;
    trsin.segmentlist = (int *)segments.data() ;

    triangulate(triswitches, &trsin, &trsout, NULL) ;

    out_pts = Eigen::Map< triangulation::pts_matrix_t >(trsout.pointlist, trsout.numberofpoints, 2) ;

    for( size_t i=0, k=0 ; i<trsout.numberoftriangles ; i++ )
    {
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
    }

    out_attrs = Eigen::Map< triangulation::attribute_matrix_t >(trsout.pointattributelist, trsout.numberofpoints, n_attrs) ;

    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;
    free(trsout.pointattributelist) ;
}


void triangulateConstraint(const triangulation::pts_matrix_t &pts, const vector<int> &segments, double area,
                       triangulation::pts_matrix_t &out_pts, std::vector<uint32_t> &triangles) {

    assert( pts.cols() == 2 ) ;
    assert( area > 0 ) ;

    size_t n_segments = segments.size()/2 ;
    size_t n_pts = pts.rows() ;

    char triswitches[20] ;
    sprintf(triswitches, "zqpQa%f", area) ;

    struct triangulateio trsin, trsout ;

    memset(&trsin, 0, sizeof(trsin)) ;
    memset(&trsout, 0, sizeof(trsin)) ;

    trsin.numberofpoints = n_pts ;
    trsin.pointlist = (REAL *)pts.data() ;
    trsin.numberofsegments = n_segments ;
    trsin.segmentlist = (int *)segments.data() ;

    triangulate(triswitches, &trsin, &trsout, NULL) ;

    out_pts = Eigen::Map< triangulation::pts_matrix_t >(trsout.pointlist, trsout.numberofpoints, 2) ;

    for( size_t i=0, k=0 ; i<trsout.numberoftriangles ; i++ )
    {
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
    }

    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;
}

void triangulateConstraint(const triangulation::pts_matrix_t &pts, const vector<int> &segments,
                       std::vector<uint32_t> &triangles) {

    assert( pts.cols() == 2 ) ;

    size_t n_segments = segments.size() ;
    size_t n_pts = pts.rows() ;

    char triswitches[20] ;
    sprintf(triswitches, "zpQ") ;

    struct triangulateio trsin, trsout ;

    memset(&trsin, 0, sizeof(trsin)) ;
    memset(&trsout, 0, sizeof(trsin)) ;

    trsin.numberofpoints = n_pts ;
    trsin.pointlist = (REAL *)pts.data() ;
    trsin.numberofsegments = n_segments ;
    trsin.segmentlist = (int *)segments.data() ;

    triangulate(triswitches, &trsin, &trsout, NULL) ;

    for( size_t i=0, k=0 ; i<trsout.numberoftriangles ; i++ )
    {
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
        triangles.push_back(trsout.trianglelist[k++]) ;
    }

    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;

}


void triangulatePolygon(const triangulation::pts_matrix_t &pts, size_t np, const triangulation::attribute_matrix_t &attrs, double area,
                       triangulation::pts_matrix_t &out_pts, triangulation::attribute_matrix_t &out_attrs, std::vector<uint32_t> &triangles) {
    assert( pts.rows() >= np ) ;
    triangulateConstraint(pts, make_poly_segments(np), attrs, area, out_pts, out_attrs, triangles);
}

void triangulatePolygon(const triangulation::pts_matrix_t &pts, size_t np, double area,
                       triangulation::pts_matrix_t &out_pts, std::vector<uint32_t> &triangles) {
    assert( pts.rows() >= np ) ;
    triangulateConstraint(pts, make_poly_segments(np), area, out_pts, triangles);
}

void triangulatePolygon(const triangulation::pts_matrix_t &pts, size_t np,
                       std::vector<uint32_t> &triangles) {
    assert( pts.rows() >= np ) ;
    triangulateConstraint(pts, make_poly_segments(np), triangles);
}



void convexHull(const triangulation::pts_matrix_t &pts, vector<uint32_t> &hull) {

    assert( pts.cols() == 2 ) ;

    size_t n_pts = pts.rows() ;

    char triswitches[20] ;
    sprintf(triswitches, "zcqQY") ;

    struct triangulateio trsin, trsout ;

    memset(&trsin, 0, sizeof(trsin)) ;
    memset(&trsout, 0, sizeof(trsin)) ;

    trsin.numberofpoints = n_pts ;
    trsin.pointlist = (REAL *)pts.data() ;

    triangulate(triswitches, &trsin, &trsout, NULL) ;

    map<uint32_t, uint32_t> segments ;
    uint32_t orig, vtx ;

    for(size_t i=0, k=0 ; i<trsout.numberofsegments ; i++) {
        int i1 = trsout.segmentlist[k++] ;
        int i2 = trsout.segmentlist[k++] ;

        if ( trsout.segmentmarkerlist[i] == 1 ) {
            if ( i == 0 ) orig = i1 ;
            segments[i1] = i2 ;
        }
    }

    vtx = orig ;

    do {
        hull.push_back(vtx) ;
        vtx = segments[vtx] ;
    } while ( vtx != orig ) ;


    free(trsout.pointlist) ;
    free(trsout.pointmarkerlist) ;
    free(trsout.trianglelist) ;
    free(trsout.segmentlist) ;
    free(trsout.segmentmarkerlist) ;

}


}}

