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
#include "location.h"


class OccupancyGrid
{
	public:
		typedef double Probability;
		
		OccupancyGrid(std::size_t size = 100, Probability p0 = 0.5);
		
		Probability update(MapLocation xt, vector<PolarCoords> zt);
		
		void rangeSensorUpdate(const PolarCoords & point);
		
		Probability inverseRangeSensorModel(std::size_t x, std::size_t y,
			const MapLocation & xt, const MapLocation & zt);
	
	private:
		Probability _l0;
		std::vector<std::vector<Probability>> _grid;
}

#endif