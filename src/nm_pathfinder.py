# MESH REFERENCE
# Mesh: {
#    "boxes": [
#          (x1, x2, y1, y2), ...  # box coordinates
#               ],
#   "adj":  {
#            (x1, x2, y1, y2): [(a1, a2, b1, b2), (c1, c2, d1, d2)], ...  # edges between boxes
#           }

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

    path = []
    boxes = {} # boxes already explored
    detail_point = {} # (box: detail point)

    print(source_point, destination_point)
    
    # Find the boxes that contain the source and destination points
    starting_box = find_box(source_point, mesh) # starting box
    destination_box = find_box(destination_point, mesh) # destination box

    queue = [(starting_box)] # queue of boxes to explore, and other info added as needed
    boxes[starting_box] = None # mark the starting box as explored, recording parent as None
    detail_point[starting_box] = source_point  # Store the midpoint of the starting box
    
    while queue:
        current_box = queue.pop(0)

        if current_box != starting_box:
            detail_point[current_box] = calculate_detail_point(current_box, boxes[current_box], detail_point[boxes[current_box]])

        if current_box == destination_box:
            # If we reached the destination box, backtrack to find the path
            while current_box is not None:
                path.append(detail_point[current_box])  # Append the xy-coordinates of the box
                current_box = boxes[current_box]
            path.reverse()  # Reverse the path to get it from source to destination  
            path.append(destination_point)  # Append the destination point
            break

        for adjacent_box in mesh["adj"][current_box]:
            if adjacent_box not in boxes:
                queue.append(adjacent_box)
                boxes[adjacent_box] = current_box
                
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
    x1, x2, y1, y2 = box1
    x3, x4, y3, y4 = box2

    x_coord = 0
    y_coord = 0
    
    # Calculate the midpoint between the two boxes
    x_range = (max(x1, x3), min(x2, x4))
    y_range = (max(y1, y3), min(y2, y4))

    if point[0] < x_range[0]:
        x_coord = x_range[0]
    elif point[0] > x_range[1]:
        x_coord = x_range[1]
    else:
        x_coord = point[0]

    if point[1] < y_range[0]:
        y_coord = y_range[0]
    elif point[1] > y_range[1]:
        y_coord = y_range[1]
    else:
        y_coord = point[1]
    
    return (x_coord, y_coord)