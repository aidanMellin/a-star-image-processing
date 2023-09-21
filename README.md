# A* Image Processing

This project implements the A* algorithm for image processing. It uses the Python Imaging Library (PIL) to open and manipulate images, and uses a priority queue to efficiently process nodes.

## How it Works

The program begins by converting the mpp.txt file and opening the image. It then reprioritizes the given list of terrain with colors. The data involved with the decision making is stored in a large data array, where the scheme is something like [(x,y), distance from parent node, heuristic value].

The program calculates 3D Distance with all x, y, and z values. It also implements a heuristic by multiplying the terrain modifier into the distance between the x and y values. The heuristic values are stored in a lookup table with the (x,y) coordinates because these heuristics are static and don't have to be computed multiple times.

The A* algorithm is then implemented. The program uses a dict of the coordinates in a tuple as a key, and the value containing a list of neighbors with their coordinates and distance from the parent node. This allows the use of the Queues.priorityQueue instead of having to build a custom one from scratch.

Finally, the program calculates the straight distance between every point in the path taken (adding between each node).

## Test Cases

The program has been tested with a variety of cases, including forced routing through water and flat elevation. The expected path is a straight line, and the distance should match exactly (minus precision error). The expected run time is less than 10 seconds.

## Elevation Output

The program also handles elevation output. It includes three separate entries for Strip Elevation due to pixel value discrepancies.