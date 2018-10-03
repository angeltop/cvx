#include "intersect.hpp"

using namespace Eigen ;

namespace cvx { namespace viz { namespace detail {

bool rayIntersectsAABB(const Ray &r, const AABB &box, float &t) {
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    tmin = (box.bounds_[r.sign_[0]].x() - r.orig_.x()) * r.invdir_.x();
    tmax = (box.bounds_[1-r.sign_[0]].x() - r.orig_.x()) * r.invdir_.x();
    tymin = (box.bounds_[r.sign_[1]].y() - r.orig_.y()) * r.invdir_.y();
    tymax = (box.bounds_[1-r.sign_[1]].y() - r.orig_.y()) * r.invdir_.y();

    if ((tmin > tymax) || (tymin > tmax)) return false;

    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    tzmin = (box.bounds_[r.sign_[2]].z() - r.orig_.z()) * r.invdir_.z();
    tzmax = (box.bounds_[1-r.sign_[2]].z() - r.orig_.z()) * r.invdir_.z();

    if ((tmin > tzmax) || (tzmin > tmax)) return false;

    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    t = tmin ;
    if ( t < 0 ) {
        t = tmax ;
        if ( t < 0 ) return false;
    }

    return true;
}

bool rayIntersectsTriangle(const Ray ray,
                           const Vector3f &v0,
                           const Vector3f &v1,
                           const Vector3f &v2,
                           bool back_face_culling,
                           float &t)
{
    const float eps = 1.0e-30;
    Vector3f edge1, edge2, h, s, q;
    float a,f,u,v;
    edge1 = v1 - v0;
    edge2 = v2 - v0;
    h = ray.dir_.cross(edge2);
    a = edge1.dot(h);
    f = 1/a;

    if ( back_face_culling ) {
        if ( a < eps ) return false;
        s = ray.orig_ - v0;
        u =  ( s.dot(h) );
        if ( u < 0.0 || u > a ) return false;
        q = s.cross(edge1);
        v = ray.dir_.dot(q) ;
        if (v < 0.0 || u + v > a) return false;

        // At this stage we can compute t to find out where the intersection point is on the line.
        t = f * edge2.dot(q) ;
    }
    else {
        if ( a > -eps && a < eps ) return false;
        s = ray.orig_ - v0;
        u = f * ( s.dot(h) );
        if ( u < 0.0 || u > 1.0 ) return false;
        q = s.cross(edge1);
        v = f * ray.dir_.dot(q) ;
        if (v < 0.0 || u + v > 1.0)
            return false;

        // At this stage we can compute t to find out where the intersection point is on the line.
        t = f * edge2.dot(q);

    }

    return t > eps ;

}

bool rayIntersectsSphere(const Ray &ray, const Vector3f &center, float radius, float &t) {

    float t0, t1; // solutions for t if the ray intersects

    float radius2 = radius * radius ;
    Vector3f L = center - ray.orig_;
    float tca = L.dot(ray.dir_);
    // if (tca < 0) return false;
    float d2 = L.dot(L) - tca * tca;
    if (d2 > radius2) return false;
    float thc = sqrt(radius2 - d2);
    t0 = tca - thc;
    t1 = tca + thc;

    if (t0 > t1) std::swap(t0, t1);

    t = t0 ;
    if ( t < 0 ) {
        t = t1; // if t0 is negative, let's use t1 instead
        if ( t < 0 ) return false; // both t0 and t1 are negative
    }

    return true;
}


/********************************************************/
/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-Möller                              */
/* Function: int triBoxOverlap(float boxcenter[3],      */
/*          float boxhalfsize[3],float triverts[3][3]); */
/* History:                                             */
/*   2001-03-05: released the code in its first version */
/*   2001-06-18: changed the order of the tests, faster */
/*                                                      */
/* Acknowledgement: Many thanks to Pierre Terdiman for  */
/* suggestions and discussions on how to optimize code. */
/* Thanks to David Hunt for finding a ">="-bug!         */
/********************************************************/

// https://gist.github.com/Philipp-M/e5747bd5a4e264ab143824059d21c120

inline void findMinMax(float x0, float x1, float x2, float &min, float &max) {
    min = max = x0;
    if (x1 < min) min = x1;
    if (x1 > max) max = x1;
    if (x2 < min) min = x2;
    if (x2 > max) max = x2;
}

inline bool planeBoxOverlap(const Vector3f &normal, const Vector3f &vert, const Vector3f &maxbox) {
    Vector3f vmin, vmax;
    float v;
    for (int q = 0; q < 3; q++) {
        v = vert[q];
        if (normal[q] > 0.0f) {
            vmin[q] = -maxbox[q] - v;
            vmax[q] = maxbox[q] - v;
        } else {
            vmin[q] = maxbox[q] - v;
            vmax[q] = -maxbox[q] - v;
        }
    }
    if ( normal.dot(vmin) > 0.0f) return false;
    if ( normal.dot(vmax) >= 0.0f) return true;

    return false;
}

/*======================== X-tests ========================*/


inline bool axisTestX01(float a, float b, float fa, float fb, const Vector3f &v0,
                        const Vector3f &v2, const Vector3f &boxhalfsize, float &rad, float &min,
                        float &max, float &p0, float &p2) {
    p0 = a * v0.y() - b * v0.z();
    p2 = a * v2.y() - b * v2.z();
    if (p0 < p2) {
        min = p0;
        max = p2;
    } else {
        min = p2;
        max = p0;
    }
    rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
    if (min > rad || max < -rad)
        return false;
    return true;
}


inline bool axisTestX2(float a, float b, float fa, float fb, const Vector3f &v0,
                       const Vector3f &v1, const Vector3f &boxhalfsize, float &rad, float &min,
                       float &max, float &p0, float &p1) {
    p0 = a * v0.y() - b * v0.z();
    p1 = a * v1.y() - b * v1.z();
    if (p0 < p1) {
        min = p0;
        max = p1;
    } else {
        min = p1;
        max = p0;
    }
    rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
    if (min > rad || max < -rad)
        return false;
    return true;
}

/*======================== Y-tests ========================*/

inline bool axisTestY02(float a, float b, float fa, float fb, const Vector3f &v0,
                        const Vector3f &v2, const Vector3f &boxhalfsize, float &rad, float &min,
                        float &max, float &p0, float &p2) {
    p0 = -a * v0.x() + b * v0.z();
    p2 = -a * v2.x() + b * v2.z();
    if (p0 < p2) {
        min = p0;
        max = p2;
    } else {
        min = p2;
        max = p0;
    }
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
    if (min > rad || max < -rad)
        return false;
    return true;
}

inline bool axisTestY1(float a, float b, float fa, float fb, const Vector3f &v0,
                       const Vector3f &v1, const Vector3f &boxhalfsize, float &rad, float &min,
                       float &max, float &p0, float &p1) {
    p0 = -a * v0.x() + b * v0.z();
    p1 = -a * v1.x() + b * v1.z();
    if (p0 < p1) {
        min = p0;
        max = p1;
    } else {
        min = p1;
        max = p0;
    }
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
    if (min > rad || max < -rad)
        return false;
    return true;
}


/*======================== Z-tests ========================*/
inline bool axisTestZ12(float a, float b, float fa, float fb, const Vector3f &v1, const Vector3f &v2,
                        const Vector3f &boxhalfsize, float &rad, float min, float max, float &p1, float &p2) {
    p1 = a * v1.x() - b * v1.y() ;
    p2 = a * v2.x() - b * v2.y();

    if ( p2 < p1 ) {
        min = p2 ;
        max = p1 ;
    } else {
        min = p1 ;
        max = p2 ;
    }
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
    if (min > rad || max < -rad)
        return false;
    return true;
}

inline bool axisTestZ0(float a, float b, float fa, float fb, const Vector3f &v0,
                       const Vector3f &v1, const Vector3f &boxhalfsize, float &rad, float &min,
                       float &max, float &p0, float &p1) {
    p0 = a * v0.x() - b * v0.y();
    p1 = a * v1.x() - b * v1.y();
    if (p0 < p1) {
        min = p0;
        max = p1;
    } else {
        min = p1;
        max = p0;
    }
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
    if (min > rad || max < -rad)
        return false;
    return true;
}

bool triangleInsideBox(const Eigen::Vector3f &tv0, const Eigen::Vector3f &tv1,
                   const Eigen::Vector3f &tv2, const Eigen::Vector3f &boxcenter, const Eigen::Vector3f &boxhalfsize) {
    Vector3f v0, v1, v2;
    float min, max ;

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */
    v0 = tv0 - boxcenter;
    v1 = tv1 - boxcenter;
    v2 = tv2 - boxcenter;

    if ( v0.x() < -boxhalfsize.x() || v0.x() > boxhalfsize.x() ) return false ;
    if ( v1.x() < -boxhalfsize.x() || v1.x() > boxhalfsize.x() ) return false ;
    if ( v2.x() < -boxhalfsize.x() || v2.x() > boxhalfsize.x() ) return false ;

    if ( v0.y() < -boxhalfsize.y() || v0.y() > boxhalfsize.y() ) return false ;
    if ( v1.y() < -boxhalfsize.y() || v1.y() > boxhalfsize.y() ) return false ;
    if ( v2.y() < -boxhalfsize.y() || v2.y() > boxhalfsize.y() ) return false ;

    if ( v0.z() < -boxhalfsize.z() || v0.z() > boxhalfsize.z() ) return false ;
    if ( v1.z() < -boxhalfsize.z() || v1.z() > boxhalfsize.z() ) return false ;
    if ( v2.z() < -boxhalfsize.z() || v2.z() > boxhalfsize.z() ) return false ;

    return true ;
}

bool triangleOutsideBox(const Eigen::Vector3f &tv0, const Eigen::Vector3f &tv1,
                   const Eigen::Vector3f &tv2, const Eigen::Vector3f &boxcenter, const Eigen::Vector3f &boxhalfsize) {

    float min, max ;
    Vector3f v0 = tv0 - boxcenter;
    Vector3f v1 = tv1 - boxcenter;
    Vector3f v2 = tv2 - boxcenter;

    /* test in X-direction */
    findMinMax(v0.x(), v1.x(), v2.x(), min, max);
    if (min > boxhalfsize.x() || max < -boxhalfsize.x())
        return true;

    /* test in Y-direction */
    findMinMax(v0.y(), v1.y(), v2.y(), min, max);
    if (min > boxhalfsize.y() || max < -boxhalfsize.y())
        return true;

    /* test in Z-direction */
    findMinMax(v0.z(), v1.z(), v2.z(), min, max);
    if (min > boxhalfsize.z() || max < -boxhalfsize.z())
        return true;

    return false;

}

bool triangleIntersectsBox(const Eigen::Vector3f &tv0, const Eigen::Vector3f &tv1,
                   const Eigen::Vector3f &tv2, const Eigen::Vector3f &boxcenter, const Eigen::Vector3f &boxhalfsize) {
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
    /*       this gives 3x3=9 more tests */
    Vector3f v0, v1, v2;
    float min, max, p0, p1, p2, rad, fex, fey, fez;
    Vector3f normal, e0, e1, e2;

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */
    v0 = tv0 - boxcenter;
    v1 = tv1 - boxcenter;
    v2 = tv2 - boxcenter;

    /* compute triangle edges */
    e0 = v1 - v0;
    e1 = v2 - v1;
    e2 = v0 - v2;



    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    fex = fabsf(e0.x());
    fey = fabsf(e0.y());
    fez = fabsf(e0.z());

    if (!axisTestX01(e0.z(), e0.y(), fez, fey, v0, v2, boxhalfsize, rad, min, max, p0, p2))
        return false;
    if (!axisTestY02(e0.z(), e0.x(), fez, fex, v0, v2, boxhalfsize, rad, min, max, p0, p2))
        return false;
    if (!axisTestZ12(e0.y(), e0.x(), fey, fex, v1, v2, boxhalfsize, rad, min, max, p1, p2))
        return false;

    fex = fabsf(e1.x());
    fey = fabsf(e1.y());
    fez = fabsf(e1.z());

    if (!axisTestX01(e1.z(), e1.y(), fez, fey, v0, v2, boxhalfsize, rad, min, max, p0, p2))
        return false;
    if (!axisTestY02(e1.z(), e1.x(), fez, fex, v0, v2, boxhalfsize, rad, min, max, p0, p2))
        return false;
    if (!axisTestZ0(e1.y(), e1.x(), fey, fex, v0, v1, boxhalfsize, rad, min, max, p0, p1))
        return false;

    fex = fabsf(e2.x());
    fey = fabsf(e2.y());
    fez = fabsf(e2.z());

    if (!axisTestX2(e2.z(), e2.y(), fez, fey, v0, v1, boxhalfsize, rad, min, max, p0, p1))
        return false;
    if (!axisTestY1(e2.z(), e2.x(), fez, fex, v0, v1, boxhalfsize, rad, min, max, p0, p1))
        return false;
    if (!axisTestZ12(e2.y(), e2.x(), fey, fex, v1, v2, boxhalfsize, rad, min, max, p1, p2))
        return false;


    /* Bullet 1: */
    // Test if triangle is completely outside box

    /* test in X-direction */
    findMinMax(v0.x(), v1.x(), v2.x(), min, max);
    if (min > boxhalfsize.x() || max < -boxhalfsize.x())
        return false;

    /* test in Y-direction */
    findMinMax(v0.y(), v1.y(), v2.y(), min, max);
    if (min > boxhalfsize.y() || max < -boxhalfsize.y())
        return false;

    /* test in Z-direction */
    findMinMax(v0.z(), v1.z(), v2.z(), min, max);
    if (min > boxhalfsize.z() || max < -boxhalfsize.z())
        return false;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    normal = e0.cross(e1) ;
    if (!planeBoxOverlap(normal, v0, boxhalfsize))
        return false;


    return true; /* box and triangle overlaps */
}

}}}
