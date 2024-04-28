#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Point.h"
#ifndef POSITION_H
#define POSITION_H

struct Point3D
{
public:
    float x = __FLT_MAX__;
    float y = __FLT_MAX__;
    float z = __FLT_MAX__;
    Point3D(){};
    Point3D(float x, float y, float z):x(x),y(y),z(z){};
    Point3D(const geometry_msgs::Point& p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
    }
    Point3D& operator-=(const Point3D& other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    } 
    Point3D operator-(const Point3D& other)
    {
        Point3D newpoint = Point3D(x - other.x, y - other.y, z - other.z);
        return newpoint;
    }
    Point3D& operator+=(const Point3D& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    float DistanceSqrt()
    {
        return x*x + y*y + z*z;
    }
};
namespace BT
{
    template <> inline Point3D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Point3D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            output.z     = convertFromString<double>(parts[2]);
            return output;
        }
    }
} // end namespace BT
#endif