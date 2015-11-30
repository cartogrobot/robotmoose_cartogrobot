/*
 * location.cpp
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Robot location representation
*/

#include "localization_point_normal.h"


/* 
 * MapLocation
 * Stores a point as cartesian coordinates and a normal vector as angle
*/

// Defualt ctor
MapLocation::MapLocation(): _x(0.0), _y(0.0), _angle(0.0) {}

// Constructs MapPoint with the given parameters
MapLocation::MapLocation(double x, double y, const Angle & angle): _x(x), _y(y), _angle(angle) {}

// Constructs MapPoint from local polar coordinates and MapPoint for their origin
MapLocation::MapLocation(const PolarCoordinates & polarCoords){
    // Correct for robot orientation
    _angle = polarCoords._angle;
    
    // Convert to cartesian and add vectors
    _x = std::cos(_angle) * polarCoords._r;
    _y = std::sin(_angle) * polarCoords._r;
}


// Constructs MapPoint from local polar coordinates and MapPoint for their origin
MapLocation::MapLocation(const PolarCoordinates & polarCoords, const MapLocation & location){
    // Correct for robot orientation
    _angle = polarCoords._angle + location._angle;
    
    // Convert to cartesian and add vectors
    _x = std::cos(_angle) * polarCoords._r + location._x;
    _y = std::sin(_angle) * polarCoords._r + location._y;
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
