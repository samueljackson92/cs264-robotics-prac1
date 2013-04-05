

#include <iostream>
#include <vector>

#include "occupancygrid.h"

int main ()
{
  OccupancyGrid grid;
  double x = -7, y = -7;
  grid.Init(x,y);

  Cell* c = grid.GetCell(1,1);
  grid.SetCellValue(1,1, 10.0);
  std::cout << *c << std::endl;

  //grid.PrintDebug();

  grid.GetCellValue(-1,-1);
  std::cout << *grid.GetCell(7,7) << std::endl;

  grid.PrintDebug();


}