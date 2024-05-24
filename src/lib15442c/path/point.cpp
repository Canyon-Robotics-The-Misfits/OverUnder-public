#include "point.hpp"

using namespace lib15442c;

Point::Point(float x, float y) :
    x(x), y(y) {};

// Point + Point operators
Point Point::operator+(const Point& rhs)
{
    return Point(x + rhs.x, y + rhs.y);
}
Point Point::operator-(const Point& rhs)
{
    return Point(x - rhs.x, y - rhs.y);
}
Point Point::operator*(const Point& rhs)
{
    return Point(x * rhs.x, y * rhs.y);
}
Point Point::operator/(const Point& rhs)
{
    return Point(x / rhs.x, y / rhs.y);
}

// Point + Point assignment operators
void Point::operator+=(const Point& rhs)
{
    x += rhs.x;
    y += rhs.y;
}
void Point::operator-=(const Point& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
}
void Point::operator*=(const Point& rhs)
{
    x *= rhs.x;
    y *= rhs.y;
}
void Point::operator/=(const Point& rhs)
{
    x /= rhs.x;
    y /= rhs.y;
}

// Point + float operators
Point Point::operator*(const float& rhs)
{
    return Point(x * rhs, y * rhs);
}
Point Point::operator/(const float& rhs)
{
    return Point(x / rhs, y / rhs);
}
void Point::operator*=(const float& rhs)
{
    x *= rhs;
    y *= rhs;
}
void Point::operator/=(const float& rhs)
{
    x /= rhs;
    y /= rhs;
}

Pose::Pose(float x, float y, float angle) :
    Point(x, y), angle(angle) {};
Pose::Pose(Point point, float angle) :
    Point(point), angle(angle) {};

// Pose + Pose operators
Pose Pose::operator+(const Pose& rhs)
{
    return Pose(x + rhs.x, y + rhs.y, angle + rhs.angle);
}
Pose Pose::operator-(const Pose& rhs)
{
    return Pose(x - rhs.x, y - rhs.y, angle - rhs.angle);
}
Pose Pose::operator*(const Pose& rhs)
{
    return Pose(x * rhs.x, y * rhs.y, angle * rhs.angle);
}
Pose Pose::operator/(const Pose& rhs)
{
    return Pose(x / rhs.x, y / rhs.y, angle / rhs.angle);
}

// Pose + Pose assignment operators
void Pose::operator+=(const Pose& rhs)
{
    x += rhs.x;
    y += rhs.y;
    angle += rhs.angle;
}
void Pose::operator-=(const Pose& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    angle -= rhs.angle;
}
void Pose::operator*=(const Pose& rhs)
{
    x *= rhs.x;
    y *= rhs.y;
    angle *= rhs.angle;
}
void Pose::operator/=(const Pose& rhs)
{
    x /= rhs.x;
    y /= rhs.y;
    angle /= rhs.angle;
}

// Pose + float operators
Pose Pose::operator+(const float& rhs)
{
    return Pose(x + rhs, y + rhs, angle + rhs);
}
Pose Pose::operator-(const float& rhs)
{
    return Pose(x - rhs, y - rhs, angle - rhs);
}
Pose Pose::operator*(const float& rhs)
{
    return Pose(x * rhs, y * rhs, angle * rhs);
}
Pose Pose::operator/(const float& rhs)
{
    return Pose(x / rhs, y / rhs, angle / rhs);
}
void Pose::operator+=(const float& rhs)
{
    x += rhs;
    y += rhs;
    angle += rhs;
}
void Pose::operator-=(const float& rhs)
{
    x -= rhs;
    y -= rhs;
    angle -= rhs;
}
void Pose::operator*=(const float& rhs)
{
    x *= rhs;
    y *= rhs;
    angle *= rhs;
}
void Pose::operator/=(const float& rhs)
{
    x /= rhs;
    y /= rhs;
    angle /= rhs;
}