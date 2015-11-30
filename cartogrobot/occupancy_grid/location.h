/*
 * location.h
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Robot location representation
*/

#ifndef __LOCATION_H
#define __LOCATION_H

#include<cmath>
#include<utility>
#include "angle.h"

// Polar coordinates for a point
class PolarCoordinates{
    double _r;
    Angle _angle;
};

// Stores a point as cartesian coordinates and a normal vector as angle
class MapLocation {
public:
    
    // Defualt ctor
    MapLocation();
    
    // Constructs MapPoint with the given parameters
    MapLocation(double x, double y, const Angle & angle);
    
    // Constructs MapPoint from local polar coordinates and MapPoint for their origin
    MapLocation(const PolarCoordinates & polarCoords);
    
    // Constructs MapPoint from local polar coordinates and MapPoint for their origin
    MapLocation(const PolarCoordinates & polarCoords, const MapLocation & location);
    
    // Accessor for x coordinate
    double getX() const;
    
    // Accessor for y coordinate
    double getY() const;
    
    // Accessor for normal angle
    const Angle & getAngle() const;
    
    // Self modifying addition
    MapLocation & operator+=(const MapLocation & other);  
    
private:
    // Cartesian coordinates
    double _x, _y;
    
    // Angle representing normal vector
    Angle _angle;
};

// Non modifying addition
MapLocation operator+(const MapLocation & p1, const MapLocation & p2);

#endif