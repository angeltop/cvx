#ifndef __TRIANGULATE_HPP__
#define __TRIANGULATE_HPP__

#include <Eigen/Core>
#include <vector>

namespace cvx { namespace util {

// A Two-Dimensional Quality Mesh Generator and Delaunay Triangulator (J. Shewchuk)
// https://www.cs.cmu.edu/~quake/triangle.html


namespace triangulation {

// input data should be row-wise matrices
    typedef Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> pts_matrix_t ;
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> attribute_matrix_t ;
}
/*
 *  Quality Delaunay triangulation of a list of points and associated attributes.
 *  Points are stored as rows of input matrix. Vertex attributes are stored as rows of attrs matrix.
 *  A quality triangulation will generate additional points to satisfy a minimum area constraint for each triangle and avoid triangles with
 *  sharp angles. The outpout points and interpolated attributes will thus be returned in out_pts, out_attrs variables.
 *  The triangle indexes are returned in triangles variable.
 */

void triangulatePoints(const triangulation::pts_matrix_t &pts, const triangulation::attribute_matrix_t &attrs, double area,
                       triangulation::pts_matrix_t &out_pts, triangulation::attribute_matrix_t &out_attrs, std::vector<uint32_t> &triangles) ;

// same as above but with no attributes
void triangulatePoints(const triangulation::pts_matrix_t &pts, double area, triangulation::pts_matrix_t &out_pts, std::vector<uint32_t> &triangles) ;

// Delaunay triangulation of list of points
void triangulatePoints(const triangulation::pts_matrix_t &pts, std::vector<uint32_t> &triangles) ;

// Quality constraint triangulation of polygon (i.e. preserving polygon edges).
// The variable pts contains vertices of a polygon and also a list of additional points inside the polygon. The number of vertices is np.
void triangulatePolygon(const triangulation::pts_matrix_t &pts, size_t np, const triangulation::attribute_matrix_t &attrs, double area,
                       triangulation::pts_matrix_t &out_pts, triangulation::attribute_matrix_t &out_attrs, std::vector<uint32_t> &triangles) ;

// same as above but with no attributes
void triangulatePolygon(const triangulation::pts_matrix_t &pts, size_t np, double area, triangulation::pts_matrix_t &out_pts, std::vector<uint32_t> &triangles) ;

// Delaunay triangulation of polygon
void triangulatePolygon(const triangulation::pts_matrix_t &pts, size_t np, std::vector<uint32_t> &triangles) ;

// Constrained quality triangulation. The vector "segments" contains pairs of indexes (to the pts array) which define edges of the graph and
// should be retained after triangulation.
void triangulateConstraint(const triangulation::pts_matrix_t &pts, const std::vector<int> &segments, const triangulation::attribute_matrix_t &attrs, double area,
                       triangulation::pts_matrix_t &out_pts, triangulation::attribute_matrix_t &out_attrs, std::vector<uint32_t> &triangles) ;

// same as above but with no attributes
void triangulateConstraint(const triangulation::pts_matrix_t &pts, const std::vector<int> &segments, double area, triangulation::pts_matrix_t &out_pts, std::vector<uint32_t> &triangles) ;

// Delaunay triangulation of graph
void triangulateConstraint(const triangulation::pts_matrix_t &pts, const std::vector<int> &segments, std::vector<uint32_t> &triangles) ;

// Convex hull of point set. This performed a Delaunay triangulation and thus not as fast as dedicated convex hull algorithms
void convexHull(const triangulation::pts_matrix_t &pts, std::vector<uint32_t> &hull) ;

}}




#endif
