import runner
from runner import scale_points

points = [(1,400),(250,400),(499,400)]

p = scale_points(points,5)
print(p)