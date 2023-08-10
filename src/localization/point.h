#ifndef POINT_H
#define POINT_H

#include <cmath>
#include <array>

class Point : public std::array<double, 2> {
public:
    Point(double x = 0, double y = 0) {
        at(0) = x;
        at(1) = y;
    }

    double x() const {
        return at(0);
    }

    double y() const {
        return at(1);
    }

    double angle() const {
        return std::atan2(at(1), at(0));
    }

    Point translate(double dx = 0, double dy = 0) const {
        Point result;
        result.at(0) = at(0) + dx;
        result.at(1) = at(1) + dy;
        return result;
    }

    Point rotate(double angle) const {
        double cos_val = std::cos(angle);
        double sin_val = std::sin(angle);
        Point result;
        result.at(0) = cos_val * at(0) - sin_val * at(1);
        result.at(1) = sin_val * at(0) + cos_val * at(1);
        return result;
    }

    double distance(const Point& other = Point()) const {
        double dx = at(0) - other.at(0);
        double dy = at(1) - other.at(1);
        return std::sqrt(dx * dx + dy * dy);
    }
};

#endif // POINT_H
