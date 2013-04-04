

#include <iostream>
#include <vector>

#include "occupancygrid.h"

int main ()
{
  OccupancyGrid grid;
  double x = -7, y = -7;
  grid.Init(x,y);

  grid.PrintDebug();

  grid.GetCellValue(9,0);

  grid.PrintDebug();
}