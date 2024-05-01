#include "geometry_msgs/Point.h"
#include <geometry_msgs/PointStamped.h>
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
    geometry_msgs::PointStamped ConvertToPoseStamped()
    {
        geometry_msgs::PointStamped p;
        p.point.x = x;
        p.point.y = y;
        p.point.z = z;
        //Debug
        p.header.frame_id = "base_link";
        return p;
    }
    geometry_msgs::Point ConvertToPoint()
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
};
#endif