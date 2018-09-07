from functools import partial
from math import ceil, sqrt

north = lambda y, x: (y - 1, x)
south = lambda y, x: (y + 1, x)
east = lambda y, x: (y, x + 1)
west = lambda y, x: (y, x - 1)

directions = lambda y, x: [east(y, x), south(y, x), west(y, x), north(y, x)]
distance = lambda c, nxt: sqrt((c[0] - nxt[0]) ** 2 + (c[1] - nxt[1]) ** 2)

class SpiralSearch:
  def __init__(self):
    pass

  def gen_grid(self,n):
    grid_size = int(ceil(sqrt(n)))
    return [[None for _ in range(grid_size)] for _ in range(grid_size)]

  def valid_coord(self,grid, coord):
    try:
      return grid[coord[0]][coord[1]] is None
    except:
      return False

  def origin(self,size):
    adjustment = 1 if size % 2 == 0 else 0
    return (size / 2 - adjustment), (size / 2 - adjustment)

  def walk_grid(self,nums):
    grid = self.gen_grid(len(nums))
    center = self.origin(len(grid[0]))
    current_position = center
    center_distance = partial(distance, center)

    for n in nums:
      y, x = current_position
      print current_position
      grid[y][x] = n
      unseen_points = [c for c in directions(y, x) if self.valid_coord(grid, c)]
      if n != nums[-1]:
        current_position = sorted(unseen_points, key=center_distance)[0]

    return grid

  def print_grid(self,highest):
    result = self.walk_grid(range(1, highest + 1))
    for row in result:
      for col in row:
        print "{:>4}".format(col if col is not None else ''),  
      print "\n"


ss = SpiralSearch()
ss.print_grid(9)