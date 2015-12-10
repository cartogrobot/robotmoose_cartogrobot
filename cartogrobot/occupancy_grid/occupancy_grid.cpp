/*
 * occupancy_grid.h
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Probabilistic occupancy grid
*/

#include "occupancy_grid.h"

OccupancyGrid::OccupancyGrid(std::size_t size, Probability p0, Probability pFree, Probability pOcc)
{
	_l0 = std::log(p0 / (1 - p0));
	_lOcc = std::log(pOcc / (1 - pOcc));
	_lFree = std::log(pFree / (1 - pFree));		// Just guessing here, might want to change
	
	_grid = std::vector<std::vector<Probability> >(size);
	
	for(auto & row : _grid)
	{
		row = std::vector<Probability>(size);
		
		for(auto & cell : row)
		{
			cell = _l0;
		}
	}
}

OccupancyGrid::Probability OccupancyGrid::operator()(std::size_t x, std::size_t y) const
{
	return 1.0 - 1.0 / (1.0 + std::exp(_grid[x][y]));
}

std::size_t OccupancyGrid::size() const
{
	return _grid.size();
}

void OccupancyGrid::update(const MapLocation & xt, const std::vector<double> & zt)
{
	Angle angle = 0.0;
	Angle delta = 2.0 * M_PI / zt.size();	
	
	for(std::size_t i = 0; i < zt.size(); i++)
	{
		//std::cout << angle << ", ";
		if(zt[i] == 0.0)
		{
			angle -= delta;
			continue;
		}
		rangeSensorUpdate(xt, MapLocation(xt, zt[i], angle));
		angle -= delta;
	}
	//std::cout << "\n";
}

void OccupancyGrid::rangeSensorUpdate(const MapLocation & begin, const MapLocation & end)
{
	double x0 = begin.getX();
	double y0 = begin.getY(); 
	
	double x1 = end.getX();
	double y1 = end.getY();
	
	double dx = fabs(x1 - x0);
    double dy = fabs(y1 - y0);

    int x = int(floor(x0));
    int y = int(floor(y0));

    int n = 1;
    int x_inc, y_inc;
    double error;

    if (dx == 0)
    {
        x_inc = 0;
        error = std::numeric_limits<double>::infinity();
    }
    else if (x1 > x0)
    {
        x_inc = 1;
        n += int(floor(x1)) - x;
        error = (floor(x0) + 1 - x0) * dy;
    }
    else
    {
        x_inc = -1;
        n += x - int(floor(x1));
        error = (x0 - floor(x0)) * dy;
    }

    if (dy == 0)
    {
        y_inc = 0;
        error -= std::numeric_limits<double>::infinity();
    }
    else if (y1 > y0)
    {
        y_inc = 1;
        n += int(floor(y1)) - y;
        error -= (floor(y0) + 1 - y0) * dx;
    }
    else
    {
        y_inc = -1;
        n += y - int(floor(y1));
        error -= (y0 - floor(y0)) * dx;
    }

    for (; n > 0; --n)
    {
    	if(n > 1)
    	{
    		// Saw through this square, not occupied
    		_grid[x][y] += _lFree - _l0;
    	}
        else
        {
        	// Object detected in this square, occupied
        	_grid[x][y] += _lOcc - _l0;
        }

        if (error > 0)
        {
            y += y_inc;
            error -= dx;
        }
        else
        {
            x += x_inc;
            error += dy;
        }
    }
}
