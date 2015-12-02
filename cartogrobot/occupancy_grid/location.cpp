/*
 * location.cpp
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Robot location representation
*/

#include "location.h"


/* 
 * MapLocation
 * Stores a point as cartesian coordinates and a normal vector as angle
*/

// Defualt ctor
MapLocation::MapLocation(): _x(0.0), _y(0.0), _angle(0.0) {}

// Constructs MapPoint with the given parameters
MapLocation::MapLocation(double x, double y, const Angle & angle): _x(x), _y(y), _angle(angle) {}

// Constructs MapPoint from local polar coordinates and MapPoint for their origin
MapLocation::MapLocation(double r, const Angle & angle){
    // Correct for robot orientation
    _angle = angle;
    
    // Convert to cartesian and add vectors
    _x = std::cos(_angle) * r;
    _y = std::sin(_angle) * r;
}


// Constructs MapPoint from local polar coordinates and MapPoint for their origin
MapLocation::MapLocation(const MapLocation & location, double r, const Angle & angle){
    // Correct for robot orientation
    _angle = angle + location._angle;
    
    // Convert to cartesian and add vectors
    _x = std::cos(_angle) * r + location._x;
    _y = std::sin(_angle) * r + location._y;
}

// Self modifying addition
MapLocation & MapLocation::operator+=(const MapLocation & other){
    this->_x += other._x;
    this->_y += other._y;
    this->_angle += other._angle;
    
    return *this;
}

// Non modifying addition
MapLocation operator+(const MapLocation & p1, const MapLocation & p2){
    return MapLocation(p1) += p2;
}

// Accessor for x coordinate
double MapLocation::getX() const{
    return _x;
}

// Accessor for y coordinate
double MapLocation::getY() const{
    return _y;
}

// Accessor for normal angle
const Angle & MapLocation::getAngle() const{
    return _angle;
}
