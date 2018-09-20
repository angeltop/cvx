#include <cvx/util/geometry/point.hpp>
#include <cvx/util/geometry/triangulate.hpp>

#include <iostream>
#include <fstream>

using namespace std ;
using namespace cvx::util ;


void save_mesh(const triangulation::pts_matrix_t &pts, const vector<uint32_t> &triangles, const string &file_name) {

    std::ofstream strm(file_name.c_str()) ;

    for( uint i=0 ; i<pts.rows() ; i++ ) {
        strm << "v " << pts.row(i) <<  ' ' << 0 << endl ;
    }

    for( uint k=0 ; k<triangles.size() ; ) {
        strm << "f " << 1 + triangles[k++] << ' ' << 1 + triangles[k++] << ' ' << 1 + triangles[k++] << endl ;
    }
}

int main(int argc, char *argv[]) {

    using triangulation::pts_matrix_t ;
    using triangulation::attribute_matrix_t ;

    pts_matrix_t pts(6, 2) ;
    attribute_matrix_t attrs(6, 1) ;

    pts << 0, 0,  -0.5, 0.5,  0, 1,  0.4, 0.4,  1, 0,  0.2, 0.2 ;
    attrs << 10, 20, 30, 40, 50, 60 ;

    {
        vector<uint32_t> triangles ;
        pts_matrix_t out_pts ;
        attribute_matrix_t out_attr ;

        triangulatePoints(pts, attrs, 0.1, out_pts, out_attr, triangles) ;
        save_mesh(out_pts, triangles, "/tmp/mesh_ptqa.obj") ;

    }

    {
        vector<uint32_t> triangles ;
        pts_matrix_t out_pts ;

        triangulatePoints(pts, 0.1, out_pts, triangles) ;
        save_mesh(out_pts, triangles, "/tmp/mesh_ptq.obj") ;
    }

    {
        vector<uint32_t> triangles ;

        triangulatePoints(pts, triangles) ;
        save_mesh(pts, triangles, "/tmp/mesh_pt.obj") ;
    }

    {
        vector<uint32_t> triangles ;
        pts_matrix_t out_pts ;
        attribute_matrix_t out_attr ;

        triangulatePolygon(pts, 5, attrs, 0.1, out_pts, out_attr, triangles) ;
        save_mesh(out_pts, triangles, "/tmp/mesh_plqa.obj") ;
    }

    {
        vector<uint32_t> triangles ;
        pts_matrix_t out_pts ;

        triangulatePolygon(pts, 5, 0.1, out_pts, triangles) ;
        save_mesh(out_pts, triangles, "/tmp/mesh_plq.obj") ;
    }

    {
        vector<uint32_t> triangles ;

        triangulatePolygon(pts, 5, triangles) ;
        save_mesh(pts, triangles, "/tmp/mesh_pl.obj") ;
    }


    vector<uint32_t> hull ;
    convexHull(pts, hull) ;
}

