# MESH REFERENCE
# Mesh: {
#    "parent": [
#          (x1, x2, y1, y2), ...  # box coordinates
#               ],
#   "adj":  {
#            (x1, x2, y1, y2): [(a1, a2, b1, b2), (c1, c2, d1, d2)], ...  # edges between parent
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
        A list of parent explored by the algorithm
    """
    start_box = find_box(source_point, mesh) # starting box
    end_box = find_box(destination_point, mesh) # destination box

    if( not start_box or not end_box): # if either the start or end box is not found, end
        print("Path not Found")
        return [], []

    # ----- VARIABLES TO RETURN -----
    path = [] # final path
    boxes = {} # boxes actually dequeued from the queue
    # -------------------------------

    # --- BIDIRECTIONAL SEARCH OBJECTS ---
    forward_discover = {start_box: None} # maps explored box to its parent, in forward direction
    backward_discover = {end_box: None} # maps explored box to its parent, in backward direction

    forward_cost = {start_box: 0}  # maps box to pathcosts (from start) in forward direction
    backward_cost = {end_box: 0}  # maps box to pathcosts (from start) in backward direction

    forward_total = {start_box: dist(source_point, destination_point)}  # maps box to total pathcosts (from start to end) forward direction
    backward_total = {end_box: dist(source_point, destination_point)}  # maps box to total pathcosts (from start to end) backward direction
    # -------------------------------------
    
    # ----------- SHARED OBJECTS -----------
    detail_point = {start_box: source_point,
                    end_box: destination_point} # maps box to its detail point
    queue = [] # queue of boxes to explore, sorted by priority (total cost)
    # --------------------------------------
    
    # ------- INITAILIZE VALUES ------- 
    heappush(queue, (dist(source_point, destination_point), start_box, "forward"))  # maintain a priority queue of boxes
    heappush(queue, (dist(destination_point, source_point), end_box, "backward"))  # format: (priority, box, direction)
    # ---------------------------------
    
    # ----------- MAIN LOOP -----------
    while queue:
        distance, current_box, direction = heappop(queue) 
        boxes[current_box] = None # Mark the current box as discovered

        if direction == "forward" and current_box in backward_discover: # if a shared box is found
            path = find_final_path(current_box, forward_discover, backward_discover, detail_point)  # find the path from source to destination
            break

        if direction == "backward" and current_box in forward_discover: # if a shared box is found
            path = find_final_path(current_box, forward_discover, backward_discover, detail_point)  # find the path from source to destination
            break
        
        # ----------- DISCOVER CHILDREN --------------
        for adjacent_box in mesh["adj"][current_box]:

            if direction == "forward": # set objects based on the direction of search
                cost = forward_cost
                cost_total = forward_total
                parent = forward_discover
                goal_point = destination_point
            elif direction == "backward":
                cost = backward_cost
                cost_total = backward_total
                parent = backward_discover
                goal_point = source_point

            # calculate the detail point & costs
            potential_detail_point = calculate_detail_point(adjacent_box, current_box, detail_point[current_box])
            cost_to_child = cost[current_box] + dist(detail_point[current_box], potential_detail_point)
            total_cost = cost_to_child + dist(potential_detail_point, goal_point)

            # add new child IF it has not been discovered or if the new cost is lower than the previously discovered cost
            if adjacent_box not in cost_total or total_cost < cost_total[adjacent_box]:
                detail_point[adjacent_box] = potential_detail_point  # update the detail point
                cost[adjacent_box] = cost_to_child  # update the cost
                parent[adjacent_box] = current_box  # set the backpointer 
                cost_total[adjacent_box] = total_cost  # update the total cost
                
                heappush(queue, (cost_total[adjacent_box], adjacent_box, direction))  # put the child on the priority queue

    if not path: # if no path was found
        print("No path found")
        return [], boxes.keys()

    return path, boxes.keys() # DONE :)

def find_box(point, mesh):
    """
    Finds the box that contains the given point in the mesh

    Args:
        point: the point to find the box for
        mesh: the mesh containing parent

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
    
    # Calculate the intersection range of the two parent
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

def find_final_path(shared_box, forward_discover, backward_discover, detail_point):
    """
    Finds the path from the source to the destination using the parent and their detail points.

    Args:
        shared_box: The current box in both forward and backward searches
        forward_discover: The parent explored in the forward direction
        backward_discover: The parent explored in the backward direction

    Returns:
        A list of points representing the path from source to destination.
    """    
    path = []  # Initialize the path list

    # ---------- FORWARD SEARCH BACKTRACKING ----------
    box = forward_discover[shared_box]  # Start from the parent of the shared box in forward search
    path.append(calculate_detail_point(shared_box, box, detail_point[shared_box])) # manually add the shared box detail point (may not exist)
    
    while box is not None:
        path.append(detail_point[box])
        box = forward_discover[box]

    # Reverse the path to get it from source to destination
    path.reverse()
    # ---------------------------------------------------
    
    # ---------- BACKWARD SEARCH BACKTRACKING ----------
    box = backward_discover[shared_box]
    path.append(calculate_detail_point(shared_box, box, detail_point[shared_box])) # manually add second detail point for shared box (may not exist)

    while box is not None:
        path.append(detail_point[box])
        box = backward_discover[box] 
    # ---------------------------------------------------  
  
    return path
