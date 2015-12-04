/*
 * occupancy_grid.h
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Probabilistic occupancy grid
*/

#ifndef __OCCUPANCY_GRID_H
#define __OCCUPANCY_GRID_H

#include <vector>
#include <limits>
#include "location.h"


class OccupancyGrid
{
	public:
		typedef double Probability;
		
		OccupancyGrid(std::size_t size = 100, Probability p0 = 0.5, Probability pfree = 0.4, Probability pOcc = 0.8);
		
		void update(const MapLocation & xt, const std::vector<double> & zt);
		
		Probability operator()(std::size_t x, std::size_t y) const;
		
		std::size_t size() const;
	
	private:
		void rangeSensorUpdate(const MapLocation & begin, const MapLocation & end);
	
		Probability _l0;
		
		std::vector<std::vector<Probability>> _grid;
		
		Probability _lOcc, _lFree;
};

#endif