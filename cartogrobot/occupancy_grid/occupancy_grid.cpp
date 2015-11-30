/*
 * occupancy_grid.h
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Probabilistic occupancy grid
*/

		
OccupancyGrid::OccupancyGrid(std::size_t size = 100, Probability p0 = 0.5):
{
	_l0 = std::log(p0 / (1-p0));

	_grid = std::vector<std::vector<Probability>>(size);
	
	for(auto & row : _grid)
	{
		row = std::vector<Probability>(size);
		
		for(auto & cell : row)
		{
			cell = initial_probability;
		}
	}
}

Probability OccupancyGrid::update(MapLocation xt, const vector<PolarCoords> & zt)
{
	for(const auto & point : zt)
	{
		rangeSensorUpdate(xt, MapLocation(zt, xt));
	}
}

void OccupancyGrid::rangeSensorUpdate(const MapLocation & xt, const MapLocation & zt)
{
	std::size_t x0 = (std::size_t) xt.getX();
	std::size_t y0 = (std::size_t) xt.getY();
	std::size_t x1 = (std::size_t) zt.getX();
	std::size_t y1 = (std::size_t) zt.getY();

	std::size_t dx = std::abs(x1 - x0);
    std::size_t dy = std::abs(y1 - y0);
    std::size_t x = x0;
    std::size_t y = y0;
    std::size_t n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n)
    {
    	// Update the probability of each grid square hit
        _grid[y][x] = _grid[y][x] + inverseRangeSensorModel(x, y, xt, zt) + _l0;

        if (error > 0)
        {
            x += x_inc;
            error -= dy;
        }
        else
        {
            y += y_inc;
            error += dx;
        }
    }
}

Probability OccupancyGrid::inverseRangeSensorModel(std::size_t x, std::size_t y,
	const MapLocatin & xt, const MapLocation & zt)
{
	if(inSquare(zt.getX(), zt.getY(), x, y))
	{
		return _lOcc;
	}
	else
	{
		return _lFree;
	}
}
