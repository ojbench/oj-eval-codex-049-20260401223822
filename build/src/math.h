#ifndef PPCA_MATH_H
#define PPCA_MATH_H

#include <iostream>
#include <cmath>
#include "config.h"

class Vec {
public:
    double x, y;

    Vec() {
        x = y = 0;
    }

    Vec(const Vec &obj) {
        x = obj.x;
        y = obj.y;
    }

    Vec(double _x, double _y) {
        x = _x, y = _y;
    }

    Vec operator+(const Vec &other) const {
        return {x + other.x, y + other.y};
    }

    Vec operator-(const Vec &other) const {
        return {x - other.x, y - other.y};
    }

    Vec operator*(double scalar) const {
        return {x * scalar, y * scalar};
    }

    Vec operator/(double scalar) const {
        return {x / scalar, y / scalar};
    }

    Vec operator-() const {
        return {-x, -y};
    }

    Vec &operator=(const Vec &other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    Vec &operator+=(const Vec &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec &operator-=(const Vec &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    friend std::istream &operator>>(std::istream &is, Vec &v) {
        is >> v.x >> v.y;
        return is;
    }

    friend std::ostream &operator<<(std::ostream &os, const Vec &v) {
        os << v.x << '\n' << v.y << '\n';
        return os;
    }

    double norm() const { // ||v||
        return std::max(std::sqrt(x * x + y * y), 1e-9);
    }

    double norm_sqr() const { // ||v||^2
        return std::max(x * x + y * y, 1e-9);
    }

    Vec normalize() const { // v / ||v||
        auto len = norm();
        return Vec(x / len, y / len);
    }

    double dot(const Vec &other) const {  // dot product the vector with other vector
        return x * other.x + y * other.y;
    }

    double cross(const Vec &other) const { // cross product the vector with other vector
        return x * other.y - y * other.x;
    }

    Vec rotate(double theta) const { // rotate the vector, the unit of theta is rad, clockwise
        double cos = std::cos(theta);
        double sin = std::sqrt(1 - cos * cos);
        return Vec{x * cos + y * sin, y * cos - x * sin};
    }
};


#endif //PPCA_MATH_H
