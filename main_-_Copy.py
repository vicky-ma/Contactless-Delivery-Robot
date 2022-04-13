import serial
import time
from gps import *
from datetime import datetime, timedelta

arduino = serial.Serial('COM4', 9600, timeout=1)

longitudeList = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -73.2, -75.657139, -75.657280, -75.1, -75.2]
latitudeList = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42.6, 45.379141, 45.379053, 45.1, 45.2]
read_long_list = []
read_lat_list = []

gpsdata = gps(mode=WATCH_ENABLE)


def getPositionData(gps):
    next = gpsdata.next()

    if next['class'] == 'TPV':
        lat = getattr(next, 'lat', "Undetected")
        long = getattr(next, 'lon', "Undetected")
        latitude = format(float(lat), '8f')
        longitude = format(float(long), '8f')
        read_long_list.append(longitude)
        read_lat_list.append(latitude)


def checkNearestLot():
    minDis = 5000
    slotNum = 0
    # get reading from gps for 4 seconds (usually get 3 pairs of data)
    start = datetime.now()
    pause = start + timedelta(seconds=4)
    while datetime.now() < pause:
        getPositionData(gpsdata)
        time.sleep(0.5)

    longi = float(read_long_list[-1])
    lati = float(read_lat_list[-1])

    for x in range(len(latitudeList)):
        longiDiff = float(longitudeList[x]) - longi
        latDiff = float(latitudeList[x]) - lati
        distance = ((longiDiff * 111000) ** 2 + (latDiff * 58000) ** 2) ** 0.5

        if distance < minDis:
            minDis = distance
            slotNum = x + 1

    return slotNum


class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    maze[end[0]][end[1]] = 0

    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []

    open_list.append(start_node)

    start_time = time.time()

    while len(open_list) > 0 and time.time() - start_time < 5:

        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > \
                    (len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(current_node, node_position)

            children.append(new_node)

        for child in children:

            for closed_child in closed_list:
                if child == closed_child:
                    continue

            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                    (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            open_list.append(child)

    maze[end[0]][end[1]] = 1
    print("ERROR: CAN NOT FIND VALID PATH")


instruction = []


def path2instruction(input_path):
    for i in range(0, len(input_path) - 1):
        instruct = (input_path[i + 1][0] - input_path[i][0], input_path[i + 1][1] - input_path[i][1])
        instruction.append(instruct)
    print(instruction)


# instruction: "int,int"
def send2arduino_step(coordinate):
    instruction_step = str(coordinate[0]) + "," + str(coordinate[1])
    received = False
    test2 = str("ck") + '\n'
    test1 = str(instruction_step).strip() + '\n'
    valid = str('T') + '\n'
    instructionValid = False

    while not received:
        arduino.write(test1.encode())
        time.sleep(.5)
        test_out = arduino.readline().decode('ascii')
        if str(test_out) == test1:
            received = True
            print(test_out)

    while not instructionValid:
        arduino.write(test2.encode())
        valid_out = arduino.readline().decode('ascii')
        if valid_out == valid:
            instructionValid = True
            print(valid_out)


def send2arduino_flow(inst):
    pathfinding = False
    while not pathfinding:
        for i in inst:
            arduino.flushInput()
            send2arduino_step(i)
            instructionDone()
        pathfinding = True
        print(pathfinding)


instDone = "instDone" + '\n'


def instructionDone():
    executeStatus = ""
    while executeStatus != instDone:
        time.sleep(.5)
        executeStatus = arduino.readline().decode('ascii')
    print(instDone)


lotCoordinates = []


def getLotCoordinates(input_maze):
    x = 0
    for i in range(0, len(input_maze)):
        x += 1
        y = 0
        for j in range(0, len(input_maze[i])):
            y += 1
            if input_maze[i][j] == 1:
                lotCoordinates.append((x - 1, y - 1))
    print(lotCoordinates)


def lotNo2Coordinate(lot_No):
    index = 0
    for i in lotCoordinates:
        index += 1
        if index == lot_No:
            return i


def coordinate2lotNo(coord):
    index = 0
    for i in lotCoordinates:
        if i == coord:
            return index
    index += 1


def ifOnTarget(currentLot, targetLot):
    if lotNo2Coordinate(currentLot) == targetLot:
        return True
    return False


maze = [

    [0, 1, 1, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0]

]


def main():
    targetLotNo = int(input())
    print(targetLotNo)
    currentLotNo = 0
    start = (0, 0)
    end = lotCoordinates[targetLotNo]
    path = astar(maze, start, end)
    path2instruction(path)
    send2arduino_flow(instruction)
    currentLotNo = checkNearestLot()
    if not ifOnTarget(currentLotNo, end):
        return False
    else:
        return True


if __name__ == '__main__':
    main()
