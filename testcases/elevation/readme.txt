Test: Elevation. 

Elevation: Mostly flat, black pixels are several orders of magnitude greater than the rest of the terrain. 

Terrain: Most of it is easy, the black pixels represent the "mountains" and there are two yellow pixels that represent starting and ending spots.

Expected path: The path should be routed around the black pixels. Distance and path do not have to match exactly.

Expected run time: <60s
Your heuristic will direct your search downward but the actual best path snakes around the black pixels. 
You will end up looking at almost every pixel so make sure you are using efficent data-structures!
