from PIL import Image, ImageDraw
import math
import time
from sys import argv

from queue import PriorityQueue

"""
make an astar program
Author: Aidan Mellin
"""


class convertData:
    def __init__(self, terrainImage, elevationFile, pathFile, outputImageFilename):
        self.terrainImage = terrainImage
        self.elevationFile = elevationFile
        self.pathFile = pathFile
        self.outputImageFile = outputImageFilename

        self.terrainType = {
            (71, 51, 3, 255): 'Paved road',
            (0, 0, 0, 255): 'Footpath',
            (248, 148, 18, 255): 'Open land',
            (255, 192, 0, 255): 'Rough meadow',
            (255, 255, 255, 255): 'Easy movement forest',
            (2, 208, 60, 255): 'Slow run forest',
            (2, 136, 40, 255): 'Walk forest',
            (5, 73, 24, 255): 'Impassible vegetation',
            (0, 0, 255, 255): 'Lake/Swamp/Marsh',
            (205, 0, 101, 255): 'Out of bounds',
            (0, 0, 0, 255) : 'Barrier',
            (205, 0, 101,): 'Strip Elevation',
            (255, 255, 255): 'Strip Elevation2',
            (0, 0, 0): 'Strip Elevation3'
        }

        self.convertElevation()

    def convertElevation(self):
        elevations = []
        with open(self.elevationFile, "r") as f:
            for line in f:
                line = line.strip().split()
                tempLine = [float(i[:-5]) for i in line]
                elevations.append(tempLine)
        self.elevations = elevations

    def makeArray(self):
        self.im = Image.open(self.terrainImage)
        px = self.im.load()
        px = self.im.convert('RGB')

        # using getpixel method

        imageSizeW, imageSizeH = px.size
        self.convertedData = [
            [0 for x in range(imageSizeW)] for y in range(imageSizeH)]

        for i in range(imageSizeH):
            for j in range(imageSizeW):
                pixVal = self.im.getpixel((j, i))
                self.convertedData[i][j] = (
                    (self.terrainType[pixVal], self.elevations[i][j]))

        return self.convertedData

    def drawPathToScreen(self, reconstPath):
        draw = ImageDraw.Draw(self.im)
        fromPoint = reconstPath[0]
        for i in reconstPath:
            toPoint = i
            draw.line([fromPoint, toPoint], fill='red', width=0)
            fromPoint = i
        self.im.show()
        self.im.save(self.outputImageFile)

    def processPath(self):
        with open(self.pathFile, 'r') as f:
            returned = []
            for line in f:
                line = line.strip().split()
                returned.append((int(line[0]), int(line[1])))
        return returned

class processData:
    def __init__(self, dataArray, start, end):
        # use these to compute the g(n)
        self.start = start
        self.end = end
        self.xDelta = 10.29
        self.yDelta = 7.55
        self.dataArray = dataArray
        self.hTree = {}

        self.createTree()

    def g(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt(math.pow((x2-x1)*self.xDelta,2) + math.pow((y2-y1)*self.yDelta,2) + math.pow((z2-z1),2))

    def h(self, n, x, y):
        H = {
            'Paved road': 1,
            'Footpath': 10,
            'Open land': 20,
            'Rough meadow': 40,
            'Easy movement forest': 50,
            'Slow run forest': 60,
            'Walk forest': 70,
            'Impassible vegetation': 90,
            'Lake/Swamp/Marsh': 99,
            'Out of bounds': math.inf,
            'Barrier': math.inf,
            'Strip Elevation': math.inf,
            'Strip Elevation2': math.inf,
            'Strip Elevation3': math.inf
        }

        distanceFromGoal = math.sqrt(math.pow(self.end[0]-x,2) + math.pow(self.end[1]-y,2))

        return distanceFromGoal * H[n]

    def createTree(self):
        self.graph = {}
        for i in range(len(self.dataArray[0])-1):
            for j in range(len(self.dataArray)-1):

                """
                x o - - - 
                o o - - -

                - o o o -
                - o x o -
                - o o o -
                """

                neighbors = self.neighbors(i, j)
                output = []
                for neighbor in neighbors:
                    x1 = i
                    y1 = j
                    z1 = self.dataArray[j][i][1]

                    x2 = neighbor[0]
                    y2 = neighbor[1]
                    z2 = neighbor[2]

                    g = self.g(x1,y1,z1,x2,y2,z2)
                    output.append(((neighbor[0], neighbor[1]), g))
                self.graph[(i, j)] = output

    def generateHTree(self):
        for x,y in self.graph.keys():
            if (x,y) not in self.hTree:
                pass

    def neighbors(self, row, col):
        result = []
        
        #Courtesy of VKen and mthurlin on stack overflow (https://stackoverflow.com/questions/1620940/determining-neighbours-of-cell-two-dimensional-list)        
        neighborCoords = lambda x, y : [(x2, y2) for x2 in range(x-1, x+2)
                                        for y2 in range(y-1, y+2)
                                        if (-1 < x <= len(self.dataArray[0]) and
                                            -1 < y <= len(self.dataArray) and
                                            (x != x2 or y != y2) and
                                            (0 <= x2 <= len(self.dataArray[0])) and
                                            (0 <= y2 <= len(self.dataArray)))]
        for x,y in neighborCoords(row,col):
            if (x,y) not in self.hTree:
                h = self.h(self.dataArray[y][x][0], x, y)
                self.hTree[(x,y)] = h
            z = self.dataArray[y][x][1]
            result.append((x, y, z))
        return result

    def computeAStar(self):
        start = self.start
        end = self.end

        distances = {node:math.inf for node in self.graph}
        for i in range(499): # Hacky way to solve the last row not being added to the tree for some reason.
            distances[(394,i)] = (math.inf)

        distances[start] = 0
        visited = set()
        cameFrom = {start: None}

        pq = PriorityQueue()
        pq.put((0 + self.hTree[start], start))

        while not pq.empty():
            _, node = pq.get()
            if node == end:
                break
            try:
                for neighbor, distance in self.graph[node]:
                    if neighbor not in visited:
                        
                            oldDistance = distances[neighbor]
                            newDistance = distances[node] + distance
                            if newDistance < oldDistance:
                                distances[neighbor] = newDistance
                                priority = newDistance + self.hTree[neighbor]
                                pq.put((priority, neighbor))
                                cameFrom[neighbor] = node
            except KeyError:
                pass
            visited.add(node)

        return cameFrom

    def reconstructPath(self, cameFrom, target):
        current = target
        path = [current]
        while cameFrom[current]:
            current = cameFrom[current]
            path.append(current)
        
        return path[::-1]

if __name__ == '__main__':
    start_time = time.time()

    if len(argv) < 5:
        tI = "terrain.png"
        eFile = "mpp.txt"
        pFile = "path.txt"
        oIF = "testOut.png"
    else:
        tI = argv[1]
        eFile = argv[2]
        pFile = argv[3]
        oIF = argv[4]

    convert = convertData(tI, eFile, pFile, oIF)
    path = convert.processPath()

    finalPath = []
    tempPath = []
    totalDistance = 0

    fromPoint = path[0]
    for i in path:
        toPoint = i
        aStar = processData(convert.makeArray(), fromPoint, toPoint)
        computed = aStar.computeAStar()
        reconstructed = aStar.reconstructPath(computed, toPoint)

        tempPath.append(reconstructed)
        fromPoint = i

    for i in tempPath:
        for j in i:
            finalPath.append(j)
    
    start = finalPath[0]
    for i in finalPath:
        totalDistance += math.sqrt(math.pow(i[0] - start[0], 2) + math.pow(i[1] - start[1], 2))
        start = i

    convert.drawPathToScreen(finalPath)

    print("--- {} seconds to run program ---\n--- Path distance: {} m ---".format((time.time() - start_time), totalDistance))
