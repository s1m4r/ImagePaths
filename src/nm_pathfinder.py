# MESH REFERENCE
# Mesh: {
#    "boxes_discovered": [
#          (x1, x2, y1, y2), ...  # box coordinates
#               ],
#   "adj":  {
#            (x1, x2, y1, y2): [(a1, a2, b1, b2), (c1, c2, d1, d2)], ...  # edges between boxes_discovered
#           }
from math import inf, sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes_discovered explored by the algorithm
    """
    # Find the boxes_discovered that contain the source and destination points
    starting_box = find_box(source_point, mesh) # starting box
    destination_box = find_box(destination_point, mesh) # destination box

    if( not starting_box or not destination_box):
        print("Path not Found")
        return [], []

    path = [] # final path
    boxes_discovered = {} # maps explored box to its parent
    boxes = {} # boxes actually dequeued from the queue

    detail_point = {} # maps box to its detail point
    start_pathcosts = {starting_box: 0}       # maps boxes_discovered to their pathcosts (found so far)
    total_pathcosts = {starting_box: dist(source_point, destination_point)}  # maps boxes_discovered to their total path costs (including distance to destination)

    queue = [] # queue of boxes_discovered to explore, and other info added as needed
    
    boxes_discovered[starting_box] = None # mark the starting box as explored, recording parent as None
    detail_point[starting_box] = source_point  # Store the detail point of the starting box
    
    heappush(queue, (dist(source_point, destination_point), starting_box))  # maintain a priority queue of cells
    
    while queue:
        distance, current_box = heappop(queue)
        boxes[current_box] = boxes_discovered[current_box]  # Store the detail point for the current box

        if current_box == destination_box:
            # If we reached the destination box, backtrack to find the path
            while current_box is not None:
                path.append(detail_point[current_box])  # Append the xy-coordinates of the box
                current_box = boxes_discovered[current_box]
            path.reverse()  # Reverse the path to get it from source to destination  
            path.append(destination_point)  # Append the destination point
            break        
        for adjacent_box in mesh["adj"][current_box]:
            potential_detail_point = calculate_detail_point(adjacent_box, current_box, detail_point[current_box])
            cost_to_child = start_pathcosts[current_box] + dist(detail_point[current_box], potential_detail_point)
            total_cost = cost_to_child + dist(potential_detail_point, destination_point)

            if adjacent_box not in start_pathcosts or total_cost < total_pathcosts[adjacent_box]:
                detail_point[adjacent_box] = potential_detail_point  # update the detail point
                start_pathcosts[adjacent_box] = cost_to_child  # update the cost
                boxes_discovered[adjacent_box] = current_box  # set the backpointer 
                total_pathcosts[adjacent_box] = total_cost  # update the total cost
                
                heappush(queue, (total_pathcosts[adjacent_box], adjacent_box))  # put the child on the priority queue
                
    # If we reach here, it means we didn't find a path
    if not path:
        print("No path found")
        return [], boxes.keys()

    return path, boxes.keys()

def find_box(point, mesh):
    """
    Finds the box that contains the given point in the mesh

    Args:
        point: the point to find the box for
        mesh: the mesh containing boxes_discovered

    Returns:
        The box that contains the point, or None if not found
    """
    
    for box in mesh["boxes"]:
        x1, x2, y1, y2 = box

        # Check if the point is within the box's boundaries
        if x1 <= point[0] <= x2 and y1 <= point[1] <= y2:
            return box
    
    return None

def calculate_detail_point(box1, box2, point):
    """
    Calculates a detail point between two boxes_discovered

    Args:
        box1: the first box
        box2: the second box

    Returns:
        A detail point (x, y) between the two boxes_discovered
    """
    # Initialize the values
    x1, x2, y1, y2 = box1
    x3, x4, y3, y4 = box2

    x_coord = 0
    y_coord = 0
    
    # Calculate the intersection range of the two boxes_discovered
    x_range = (max(x1, x3), min(x2, x4))
    y_range = (max(y1, y3), min(y2, y4))

    # determine x-coord
    if point[0] < x_range[0]:
        x_coord = x_range[0]
    elif point[0] > x_range[1]:
        x_coord = x_range[1]
    else:
        x_coord = point[0]
    
    # determine y-coord
    if point[1] < y_range[0]:
        y_coord = y_range[0]
    elif point[1] > y_range[1]:
        y_coord = y_range[1]
    else:
        y_coord = point[1]
    
    return (x_coord, y_coord)

def dist(point1, point2):
    """ Calculates the Euclidean distance between two points.

    Args:
        point1: The first point as a tuple (x1, y1).
        point2: The second point as a tuple (x2, y2).

    Returns:
        The Euclidean distance between the two points.
    """
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
