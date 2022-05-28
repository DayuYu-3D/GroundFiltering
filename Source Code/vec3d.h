/********************************************
 * Class: Vec3d
 * Author: Dayu
 * Site: WHU
 * Date: 20201207
********************************************/
#ifndef VEC3D_H
#define VEC3D_H

#include <math.h>

// a minimal vertex class of 3 doubles and overloaded math operators
class Vec3d {
public:
    Vec3d(double x, double y, double z) {
        _v[0] = x;
        _v[1] = y;
        _v[2] = z;
    }
    Vec3d() {
        _v[0] = 0.0;
        _v[1] = 0.0;
        _v[2] = 0.0;
    }

    inline double &x()
    {
        return _v[0];
    }
    inline double &y()
    {
        return _v[1];
    }
    inline double &z()
    {
        return _v[2];
    }

    /** Length of the vector = vec . vec */
    inline double length() const
    {
        return sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    }

    /** Normalize the vector so that it has length unity.
    */
    inline Vec3d normalize() const
    {
        double l = Vec3d::length();
        return Vec3d(_v[0] / l, _v[1] / l, _v[2] / l);
    }

    /** Cross product. */
    inline const Vec3d cross(const Vec3d &v) const
    {
        return Vec3d(
                   _v[1] * v._v[2] - _v[2] * v._v[1],
                   _v[2] * v._v[0] - _v[0] * v._v[2],
                   _v[0] * v._v[1] - _v[1] * v._v[0]
               );
    }

    /** Dot product. */
    inline double dot(const Vec3d &v) const
    {
        return _v[0] * v._v[0] + _v[1] * v._v[1] + _v[2] * v._v[2];
    }

    inline Vec3d &operator += (const Vec3d &rhs)
    {
        _v[0] += rhs._v[0];
        _v[1] += rhs._v[1];
        _v[2] += rhs._v[2];
        return *this;
    }

    /** Divide by scalar. */
    inline const Vec3d operator / (const double &rhs) const
    {
        return Vec3d(_v[0] / rhs, _v[1] / rhs, _v[2] / rhs);
    }

    /** Binary vector subtract. */
    inline const Vec3d operator - (const Vec3d &rhs) const
    {
        return Vec3d(_v[0] - rhs._v[0], _v[1] - rhs._v[1], _v[2] - rhs._v[2]);
    }

    /** Binary vector add. */
    inline const Vec3d operator + (const Vec3d &rhs) const
    {
        return Vec3d(_v[0] + rhs._v[0], _v[1] + rhs._v[1], _v[2] + rhs._v[2]);
    }

    /** Multiply by scalar. */
    inline const Vec3d operator * (double &rhs) const
    {
        return Vec3d(_v[0] * rhs, _v[1] * rhs, _v[2] * rhs);
    }

    /** Negation operator. Returns the negative of the Vec3d. */
    inline const Vec3d operator - () const
    {
        return Vec3d (-_v[0], -_v[1], -_v[2]);
    }

    inline double &operator [] (int i) {
        return _v[i];
    }
    inline double operator [] (int i) const {
        return _v[i];
    }

public:
    double _v[3];
};

#endif // VEC3D_H
