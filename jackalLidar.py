# This script is executed the first time the script node computes, or the next time

# it computes after this script is modified or the 'Reset' button is pressed.

#

# The following callback functions may be defined in this script:

#     setup(db): Called immediately after this script is executed

#     compute(db): Called every time the node computes (should always be defined)

#     cleanup(db): Called when the node is deleted or the reset button is pressed

# Available variables:

#    db: og.Database The node interface - attributes are exposed in a namespace like db.inputs.foo and db.outputs.bar.

#                    Use db.log_error, db.log_warning to report problems in the compute function.

#    og: The omni.graph.core module

import math

import numpy as np

import omni

import omni.usd

import asyncio

import omni.graph.core as og

from isaacsim.sensors.physx import _range_sensor 

import omni.isaac.core.utils.prims as prim_utils

from omni.isaac.core import World

from pxr import UsdGeom, Gf, Usd

import heapq



# azimuth angle referes to angle realtive to the direction the lidar is facing

# it is facing forward in front of the jackal robot

# positive angles are ccw from the 0

# negative angles are cw from the 0



# jackal variables

prim_path = "/World/jackal/base_link"

stage = omni.usd.get_context().get_stage()

jackal = stage.GetPrimAtPath(prim_path)



# lidar variables

lidar_path = "/World/jackal/base_link/sick_lms1xx_lidar_frame/Lidar"  # Replace with your lidar's path

lidar_prim = prim_utils.get_prim_at_path(lidar_path)

lidar_interface = _range_sensor.acquire_lidar_sensor_interface()



# lidar data

depth = np.empty((900, 1))

zenith = np.empty((900, 1))

azimuth = np.empty((900, 1))



# timeline

timeline = omni.timeline.get_timeline_interface()

iteration = 0



# map of the world

# each space in the 2d array represents an intersection in the isaac sim map

rows = 51

cols = 51

world = [[1 for _ in range(51)] for _ in range(51)]

origin = 25

goal = (46, 43)

pathTaken = [(origin, origin)]

optimalPath = [(origin, origin)]

reached = True

rotationError = math.radians(1)

translationError = 0.1

# a pose is a coordinate on the world

# the first number is the row or y value

# the second number is the column or x value

nextPose = [origin, origin]

nextAngle = 0

arrived = False



# wheel speed

speed = 10

turnSpeed = 10



def setup(db: og.Database):

    pass



def cleanup(db: og.Database):

    pass



def compute(db: og.Database):

    timeline.play()                                             # Start the Simulation  

    asyncio.ensure_future(get_lidar_param())                    # Only ask for data after sweep is complete

    global iteration

    global optimalPath

    global arrived



    # get robot info

    pos = getPosition()

    z_rot = math.radians(get_local_transform_xform(prim=jackal)[1][2])

    jPose = (origin - pos[0], origin - pos[1])



    # gives it time for the jackal to fall

    if 75 < iteration and iteration < 100:

        fill_map(jack_pose=jPose, jackalAngle=z_rot)

        optimalPath = a_star_search(world, [origin, origin], goal)



    if (iteration >= 100) and (not arrived):

        # x,y,z pos

        global nextAngle

        global nextPose

        global backtracking

        global reached

        global pathTaken



        if iteration % 100 == 0:

            printMap()



        if reached:

            # always 1 because that is the pose right after our current pose

            nextPose = optimalPath[1]

            nextAngle = angleToNextPose(nextPose, jPose)

            reached = False



        # figure out wheel power 

        if iteration % 10 == 0:

            nextAngle = angleToNextPose(nextPose, jPose)

        

        wheels = findWheelPower(z_rot=z_rot)

        if checkReached(jPose=jPose):

            resolveReach(jPose=jPose, z_rot=z_rot)



        db.outputs.left = wheels[0]

        db.outputs.right = wheels[1]

    else:

        db.outputs.left = 0

        db.outputs.right = 0



    iteration += 1

    return True



def checkReached(jPose):

    withinRadius = False

    withinRadius = (((jPose[0] - nextPose[0]) ** 2 + (jPose[1] - nextPose[1]) ** 2) ** 0.5) <= translationError

    return withinRadius



def resolveReach(jPose, z_rot):

    global reached

    global pathTaken

    global optimalPath

    global world

    global arrived

    

    reached = True

    pathTaken.append(nextPose)



    if (nextPose[0] == goal[0]) and (nextPose[1] == goal[1]):

        arrived = True



    fill_map(jack_pose=jPose, jackalAngle=z_rot)

    optimalPath = a_star_search(world, [int(jPose[0] + 0.5), int(jPose[1] + 0.5)], goal)# new optimal path with current location as start



# finds the wheel power needed

def findWheelPower(z_rot):

    global nextAngle

    global rotationError



    new_z = z_rot + math.pi

    new_angle = nextAngle + math.pi



    if new_z >= new_angle:

        ccwDistance = 2 * math.pi - new_z + new_angle

        cwDistance = new_z - new_angle

    else:

        ccwDistance = new_angle - new_z

        cwDistance = 2*math.pi - new_angle + new_z

    

    if nextAngle - rotationError < z_rot and z_rot < nextAngle + rotationError:

        wheels = forward()

    elif ccwDistance <= cwDistance:

        wheels = ccw()

    else:

        wheels = cw()

    return wheels



# returns the angle from the vertucal needed to go to the next pose relative to jackal_pose 

def angleToNextPose(pose, jackal_pose):

    # atan2 extends the range of atan from -pi to pi by considering the sign of the points

    # it is jackal_pose - pose because of the way that 2d arrays are numbering with increasing rows going down and increasing cols going right

    return math.atan2(jackal_pose[1] - pose[1], jackal_pose[0] - pose[0])



# returns the index of the shortest path

def findShortest():

    index = np.unravel_index(np.argmin(depth, axis=None), depth.shape)

    return index



# returns the index of the longest path

def findLongest():

    index = np.unravel_index(np.argmax(depth, axis=None), depth.shape)

    return index



# returns depth, zenith, and azimuth

async def get_lidar_param():                                    # Function to retrieve data from the LIDAR

    await omni.kit.app.get_app().next_update_async()            # wait one frame for data

    #timeline.pause()                                           # Pause the simulation to populate the LIDAR's depth buffers

    global depth

    global zenith

    global azimuth

    

    depth = lidar_interface.get_linear_depth_data(lidar_path)

    zenith = lidar_interface.get_zenith_data(lidar_path)

    azimuth = lidar_interface.get_azimuth_data(lidar_path)

    pass



# fills the world map by setting walls to -1

# an angle of 0 points straight north in the direction of the x-axis in isaacs sim

# that is why sin is used for x distance

# and cos is used for y distance

# since it is an angle from the vertical

def fill_map(jack_pose, jackalAngle):

    global world

    for ray in range(0, 900):

        distance = depth[ray, 0]

        lidarAngle = azimuth[ray]

        # since the lidar angle is alrady relative to the orientation of the jackal

        # i can simply add the angles to get the angle relative to the world

        actualAngle = lidarAngle + jackalAngle

        yDistance = distance * math.sin(actualAngle)

        xDistance = distance * math.cos(actualAngle)

        

        # xDistrance and yDistance are subtracted since the angle is from the vertical from north

        # this means that the sign is switched in order to increase row or col so you subtract to get the right sign

        # rounds for better accuracy

        xcoor = jack_pose[0] - xDistance

        ycoor = jack_pose[1] - yDistance

        

        if validCoor(xcoor) and validCoor(ycoor):

            #print("x: ", jack_pose[0] - xDistance)

            #print("y: ", jack_pose[1] - yDistance)

            world[int(xcoor + 0.5)][int(ycoor + 0.5)] = 0



# the coordinate is within the world conditions and it is close to an actual coordinate

def validCoor(coor):

    return ((coor - int(coor) <= 0.2) or (coor - int(coor) >= 0.8)) and (0 <= coor and coor < 51)



def printMap():

    for row in range(0, 51):

        for col in range(0, 51):

            print(world[row][col], end="|")

        print()

    pass



# gets the x y and z position of jackal in the world stage

def getPosition():

    jackal_pos = jackal.GetAttribute("xformOp:translate").Get()

    return jackal_pos



# returns the x y and z euler angle rotation of the jackal

def getOrientation():

    jackal_angle = jackal.GetAttribute("xformOp:rotateXYZ").Get() 

    return jackal_angle



# decomposes a matrix to return translation, rotation, and scale of prim

def decompose_matrix(mat: Gf.Matrix4d):

    reversed_ident_mtx = reversed(Gf.Matrix3d())

    translate: Gf.Vec3d = mat.ExtractTranslation()

    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in mat.ExtractRotationMatrix()))



    mat.Orthonormalize()

    rotate = Gf.Vec3d(*reversed(mat.ExtractRotation().Decompose(*reversed_ident_mtx)))

    return translate, rotate, scale



# gets the transofrmation of prim

def get_local_transform_xform(prim: Usd.Prim):

    xform = UsdGeom.Xformable(prim)

    local_transform: Gf.Matrix4d = xform.GetLocalTransformation()



    decomposed_matrix = decompose_matrix(local_transform)

    return decomposed_matrix



def get_global_transform_omni(prim):

    xform = UsdGeom.Xformable(prim)

    mtrx = xform.ComputeLocalToWorldTransform(0)

        

    decomposed_matrix = decompose_matrix(mtrx)

    return decomposed_matrix



# returns wheel speeds to make jackal go forward

def forward():

    return [speed, speed]



# returns wheel speeds to make jackal go backwards

def backward():

    return [-speed, -speed]



# returns wheel speeds to make jackal rotate clockwise in place

def cw():

    return [turnSpeed, -turnSpeed]



# returns wheel speeds to make jackal rotate counter-clockwise in place

def ccw():

    return [-turnSpeed, turnSpeed]



# implements A*

class Cell:

    def __init__(self):

        self.parent_i = 0  # Parent cell's row index

        self.parent_j = 0  # Parent cell's column index

        self.f = float('inf')  # Total cost of the cell (g + h)

        self.g = float('inf')  # Cost from start to this cell

        self.h = 0  # Heuristic cost from this cell to destination



# Check if a cell is valid (within the grid)

def is_valid(row, col):

    return (row >= 0) and (row < rows) and (col >= 0) and (col < cols)



# Check if a cell is unblocked

def is_unblocked(grid, row, col):

    return grid[row][col] == 1



# Check if a cell is the destination

def is_destination(row, col, dest):

    return row == dest[0] and col == dest[1]



# Calculate the heuristic value of a cell (Euclidean distance to destination)

def calculate_h_value(row, col, dest):

    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5



# Trace the path from source to destination

def trace_path(cell_details, dest):

    print("The Path is ")

    path = []

    row = dest[0]

    col = dest[1]



    # Trace the path from destination to source using parent cells

    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):

        path.append((row, col))

        temp_row = cell_details[row][col].parent_i

        temp_col = cell_details[row][col].parent_j

        row = temp_row

        col = temp_col



    # Add the source cell to the path

    path.append((row, col))

    # Reverse the path to get the path from source to destination

    path.reverse()



    # Print the path

    for i in path:

        print("->", i, end=" ")

    print()

    return path



# Implement the A* search algorithm

# The code is from GeeksForGeeks

def a_star_search(grid, src, dest):

    # Check if the source and destination are valid

    if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):

        print("Source or destination is invalid")

        return



    # Check if the source and destination are unblocked

    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):

        print("Source or the destination is blocked")

        return



    # Check if we are already at the destination

    if is_destination(src[0], src[1], dest):

        print("We are already at the destination")

        return



    # Initialize the closed list (visited cells)

    closed_list = [[False for _ in range(cols)] for _ in range(rows)]

    # Initialize the details of each cell

    cell_details = [[Cell() for _ in range(cols)] for _ in range(rows)]



    # Initialize the start cell details

    i = src[0]

    j = src[1]

    cell_details[i][j].f = 0

    cell_details[i][j].g = 0

    cell_details[i][j].h = 0

    cell_details[i][j].parent_i = i

    cell_details[i][j].parent_j = j



    # Initialize the open list (cells to be visited) with the start cell

    open_list = []

    heapq.heappush(open_list, (0.0, i, j))



    # Initialize the flag for whether destination is found

    found_dest = False



    # Main loop of A* search algorithm

    while len(open_list) > 0:

        # Pop the cell with the smallest f value from the open list

        p = heapq.heappop(open_list)



        # Mark the cell as visited

        i = p[1]

        j = p[2]

        closed_list[i][j] = True



        # For each direction, check the successors

        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        for dir in directions:

            new_i = i + dir[0]

            new_j = j + dir[1]



            # If the successor is valid, unblocked, and not visited

            if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:

                # If the successor is the destination

                if is_destination(new_i, new_j, dest):

                    # Set the parent of the destination cell

                    cell_details[new_i][new_j].parent_i = i

                    cell_details[new_i][new_j].parent_j = j

                    print("The destination cell is found")

                    # Trace and print the path from source to destination

                    path = trace_path(cell_details, dest)

                    found_dest = True

                    return path

                else:

                    # Calculate the new f, g, and h values

                    if dir == (1, 1) or dir == (1, -1) or dir == (-1, 1) or dir == (-1, -1):

                        g_new = cell_details[i][j].g + 1.414

                    else:

                        g_new = cell_details[i][j].g + 1.0



                    h_new = calculate_h_value(new_i, new_j, dest)

                    f_new = g_new + h_new



                    # If the cell is not in the open list or the new f value is smaller

                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:

                        # Add the cell to the open list

                        heapq.heappush(open_list, (f_new, new_i, new_j))

                        # Update the cell details

                        cell_details[new_i][new_j].f = f_new

                        cell_details[new_i][new_j].g = g_new

                        cell_details[new_i][new_j].h = h_new

                        cell_details[new_i][new_j].parent_i = i

                        cell_details[new_i][new_j].parent_j = j



    # If the destination is not found after visiting all cells

    if not found_dest:

        print("Failed to find the destination cell")
