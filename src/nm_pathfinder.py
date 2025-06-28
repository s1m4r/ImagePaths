# MESH REFERENCE
# Mesh: {
#    "boxes": [
#          (x1, x2, y1, y2), ...  # box coordinates
#               ],
#   "adj":  {
#            (x1, x2, y1, y2): [(a1, a2, b1, b2), (c1, c2, d1, d2)], ...  # edges between boxes
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
        A list of boxes explored by the algorithm
    """
    # Find the boxes that contain the source and destination points
    starting_box = find_box(source_point, mesh) # starting box
    destination_box = find_box(destination_point, mesh) # destination box

    path = [] # final path
    boxes = {} # maps explored box to its parent
    detail_point = {} # maps box to its detail point
    start_pathcosts = {starting_box: 0}       # maps boxes to their pathcosts (found so far)

    queue = [] # queue of boxes to explore, and other info added as needed
    
    boxes[starting_box] = None # mark the starting box as explored, recording parent as None
    detail_point[starting_box] = source_point  # Store the midpoint of the starting box
    
    heappush(queue, (0, starting_box))  # maintain a priority queue of cells
    
    while queue:
        distance, current_box = heappop(queue)

        if current_box == destination_box:
            # If we reached the destination box, backtrack to find the path
            while current_box is not None:
                path.append(detail_point[current_box])  # Append the xy-coordinates of the box
                current_box = boxes[current_box]
            path.reverse()  # Reverse the path to get it from source to destination  
            path.append(destination_point)  # Append the destination point
            break        

        for adjacent_box in mesh["adj"][current_box]:
            potential_detail_point = calculate_detail_point(adjacent_box, current_box, detail_point[current_box])
            cost_to_child = start_pathcosts[current_box] + dist(detail_point[current_box], potential_detail_point)
            
            if adjacent_box not in start_pathcosts or cost_to_child < start_pathcosts[adjacent_box]:
                detail_point[adjacent_box] = potential_detail_point  # update the detail point
                start_pathcosts[adjacent_box] = cost_to_child  # update the cost
                boxes[adjacent_box] = current_box  # set the backpointer 
                heappush(queue, (cost_to_child, adjacent_box))  # put the child on the priority queue
                
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
        mesh: the mesh containing boxes

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
    Calculates a detail point between two boxes

    Args:
        box1: the first box
        box2: the second box

    Returns:
        A detail point (x, y) between the two boxes
    """
    # Initialize the values
    x1, x2, y1, y2 = box1
    x3, x4, y3, y4 = box2

    x_coord = 0
    y_coord = 0
    
    # Calculate the intersection range of the two boxes
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

def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    paths = {initial_position: []}          # maps cells to previous cells on path
    pathcosts = {initial_position: 0}       # maps cells to their pathcosts (found so far)
    queue = []
    heappush(queue, (0, initial_position))  # maintain a priority queue of cells
    
    while queue:
        priority, cell = heappop(queue)
        if cell == destination:
            return path_to_cell(cell, paths)
        
        # investigate children
        for (child, step_cost) in adj(graph, cell):
            # calculate cost along this path to child
            cost_to_child = priority + transition_cost(graph, cell, child)
            if child not in pathcosts or cost_to_child < pathcosts[child]:
                pathcosts[child] = cost_to_child            # update the cost
                paths[child] = cell                         # set the backpointer
                heappush(queue, (cost_to_child, child))     # put the child on the priority queue
            
    return False

def path_to_cell(cell, paths):
    if cell == []:
        return []
    return path_to_cell(paths[cell], paths) + [cell]
    
def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    res = []
    for delta in [(x, y) for x in [-1,0,1] for y in [-1,0,1] if not (x==0 and y==0)]:
        new = (cell[0] + delta[0], cell[1] + delta[1])
        if new in level['spaces']:
            res.append((new, transition_cost(level, new, cell)))
    return res

def transition_cost(level, cell, cell2):
    distance = sqrt((cell2[0] - cell[0])**2 + (cell2[1] - cell[1])**2)
    average_cost = (level['spaces'][cell] + level['spaces'][cell2])/2
    return distance * average_cost

def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")
