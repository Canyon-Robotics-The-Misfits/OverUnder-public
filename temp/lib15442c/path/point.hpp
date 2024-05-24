#pragma once

namespace lib15442c {
class Point {
public:
    float x;
    float y;

    Point(float x, float y);

    Point operator+(const Point& rhs);
    Point operator-(const Point& rhs);
    Point operator*(const Point& rhs);
    Point operator/(const Point& rhs);

    void operator+=(const Point& rhs);
    void operator-=(const Point& rhs);
    void operator*=(const Point& rhs);
    void operator/=(const Point& rhs);

    Point operator*(const float& rhs);
    Point operator/(const float& rhs);

    void operator*=(const float& rhs);
    void operator/=(const float& rhs);
};

// Pose class that inherits from Point and adds an angle
class Pose : public Point {
public:
    float angle;

    Pose(float x, float y, float angle);
    Pose(Point point, float angle);

    Pose operator+(const Pose& rhs);
    Pose operator-(const Pose& rhs);
    Pose operator*(const Pose& rhs);
    Pose operator/(const Pose& rhs);

    void operator+=(const Pose& rhs);
    void operator-=(const Pose& rhs);
    void operator*=(const Pose& rhs);
    void operator/=(const Pose& rhs);

    Pose operator+(const float& rhs);
    Pose operator-(const float& rhs);
    Pose operator*(const float& rhs);
    Pose operator/(const float& rhs);

    void operator+=(const float& rhs);
    void operator-=(const float& rhs);
    void operator*=(const float& rhs);
    void operator/=(const float& rhs);
};
} // namespace lib15442c

#define pos(x, y) lib15442c::Point(x, y)
