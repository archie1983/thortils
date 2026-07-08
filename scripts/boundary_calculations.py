from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.prepared import prep
from scipy.spatial import KDTree
from collections import deque
import matplotlib.pyplot as plt
from ai2_thor_model_training.ae_utils import NavigationActions

class BoundaryCalculations:
    def __init__(self, grid_size=0.125):
        a = NavigationActions(step = grid_size)

    def get_room_perimeter_points_1st_pass(self, reachable_points, unreachable_points, room_of_placement, na):
        '''
        First pass for room perimeter point gathering. Here we get all points that form a boundary- either the room
        boundary or some object within the room. The idea is that we look at all reachable points and attempt to walk
        from them to any direction. If we reach a point that is unreachable or not in reachable point set, then that
        means we started walking from a boundary point.

        :param reachable_points:
        :param unreachable_points:
        :param room_of_placement:
        :param na:
        :return:
        '''
        room_polygon = prep(Polygon(room_of_placement[1]))

        reachable_room_points = {
            (x, y) for (x, y) in reachable_points if room_polygon.contains(Point(x, y))
        }

        unreachable_room_points = {
            (x, y) for (x, y) in unreachable_points if room_polygon.contains(Point(x, y))
        }

        # Test each reachable point whether it has neighbours that are not reachable
        boundary_points = set()
        for rpos in reachable_room_points:
            for move in na.STRAIGHT_MOVE_MOVES:
                new_x, new_y = na.apply(rpos[0], rpos[1], move)
                if (new_x, new_y) in unreachable_room_points or (new_x, new_y) not in reachable_room_points:
                #if (new_x, new_y) not in reachable_room_points:
                    boundary_points.add((rpos[0], rpos[1]))
                    break
        return boundary_points


    def get_room_perimeter_points_2nd_pass_ds(self, boundary_points, na):
        '''
        Second pass for room boundary points. Extracts connected components
        from the boundary points set.
        '''
        all_sub_boundaries = set()

        # Work with a copy since we'll be modifying it
        remaining_points = set(boundary_points)

        while remaining_points:
            # Start a new sub-boundary with any remaining point
            start_point = remaining_points.pop()
            current_sub_boundary = set([start_point])  # Use set for easy membership testing
            changed = True

            # Keep adding points until no new points are found
            while changed:
                changed = False

                # For each point in the current boundary
                for cbp in list(current_sub_boundary):  # Iterate over copy
                    # Check all four directions
                    for move in na.MOVE_MOVES:
                        new_x, new_y = na.apply(cbp[0], cbp[1], move)

                        # If this neighbor is in remaining_points, add it
                        if (new_x, new_y) in remaining_points:
                            remaining_points.remove((new_x, new_y))
                            current_sub_boundary.add((new_x, new_y))
                            changed = True
                        # Also check if it's in the current boundary but not yet processed
                        elif (new_x, new_y) in current_sub_boundary:
                            continue  # Already in our boundary
                        # Otherwise, it's not a boundary point or doesn't exist

            # Store as frozenset (hashable) instead of list
            all_sub_boundaries.add(frozenset(current_sub_boundary))
        return all_sub_boundaries

        # # Now find which sub-boundary is the room boundary (outermost)
        # # You can use your polygon logic here
        # room_boundary = find_outermost_boundary(all_sub_boundaries)
        #
        # return room_boundary

    def get_room_perimeter_points_2nd_pass_orig(self, boundary_points, na):
        '''
        Second pass for room boundary points. The idea is that we take any point in the boundary points from
        get_room_perimeter_points_1st_pass(...) and start walking in all directions. If we reach another point
        from the original boundary points set, then it's the point that belongs to the same boundary, so take it
        out from the starting set and keep going. Once there are no points left to reach, we have established one
        boundary. Now take any point from the remaining set and repeat the same procedure. In the end we should
        have a set of boundaries within the room. Now we form a polygon from each of those sets and check which
        one is within another one. The outer polygon is what we want.

        :param boundary_points:
        :param na:
        :return:
        '''

        boundary_points = boundary_points.copy()
        all_sub_boundaries = list()
        current_sub_boundary = set()
        neighbour_found = False

        cbp = boundary_points.pop() # take any point as a starter
        #bounary_points = list(boundary_points)
        #cbp = bounary_points[0]
        while len(boundary_points) > 0:
            current_sub_boundary.add(cbp) # add the current point to the sub-boundary we're processing
            neighbour_found = False
            for move in na.MOVE_MOVES: # walk in all directions from current point until we find another point from the boundary or exhaust all moves
                new_x, new_y = na.apply(cbp[0], cbp[1], move)
                # if a boundary point found, then remove it from the big boundary set and use it as the new current point.
                # We will add it to the current sub-boundary on next iteration of outer loop.
                if (new_x, new_y) in boundary_points:
                    boundary_points.remove((new_x, new_y))
                    #print("Removed: ", (new_x, new_y))
                    cbp = (new_x, new_y)
                    neighbour_found = True
                    break
            if not neighbour_found:
                # if we went through all moves and didn't find a new point that belongs to a boundary, then we
                # have finished traversing this sub-boundary
                print("Closed boundary: ", current_sub_boundary)
                all_sub_boundaries.append(current_sub_boundary)
                current_sub_boundary = set()
                cbp = boundary_points.pop()

        if len(boundary_points) > 0:
            all_sub_boundaries.append(boundary_points)

        if len(current_sub_boundary) > 0:
            all_sub_boundaries.append(current_sub_boundary)

        print(all_sub_boundaries)
        return all_sub_boundaries


    def find_outermost_boundary(self, all_sub_boundaries):
        '''
        Find which boundary contains all others (i.e., the room boundary).
        '''
        # Convert each frozenset to a polygon
        sub_boundary_polygons = []
        for boundary_set in all_sub_boundaries:
            if len(boundary_set) < 3:
                continue  # Need at least 3 points for a polygon

            # Sort points to form a proper polygon (you'll need to order them)
            # This is a simple approach - you might want to sort by angle around centroid
            points_list = list(boundary_set)
            # For now, we'll use the convex hull as approximation
            from shapely.geometry import Polygon
            try:
                polygon = Polygon(points_list).convex_hull
                sub_boundary_polygons.append((boundary_set, polygon))
            except:
                continue

        # Find the outermost polygon (the one that contains all others)
        for i, (set_i, poly_i) in enumerate(sub_boundary_polygons):
            is_outermost = True
            for j, (set_j, poly_j) in enumerate(sub_boundary_polygons):
                if i != j:
                    # If poly_i contains poly_j, it might be the outer boundary
                    if not poly_i.contains(poly_j):
                        is_outermost = False
                        break
            if is_outermost:
                return set_i  # Return as regular set of points

        # Fallback: return the largest boundary
        return max(all_sub_boundaries, key=len)


    def extract_room_boundary(self, perimeter_points, room_polygon, margin=0.1):
        """
        Extract the room boundary by combining:
        1. Filtering by room polygon
        2. Taking the largest continuous component
        """
        # Step 1: Filter by room polygon first
        polygon = Polygon(room_polygon)
        boundary_candidates = set()

        for point in perimeter_points:
            shapely_point = Point(point[0], point[1])
            # Keep points on or near the room boundary
            if shapely_point.distance(polygon.boundary) <= margin:
                boundary_candidates.add(point)

        # Step 2: Find the largest connected component among candidates
        if not boundary_candidates:
            return set()

        points_list = list(boundary_candidates)
        tree = KDTree(points_list)
        visited = set()
        largest_component = set()

        for i, point in enumerate(points_list):
            if i not in visited:
                component = set()
                queue = deque([point])
                visited.add(i)

                while queue:
                    current = queue.popleft()
                    component.add(current)
                    current_idx = points_list.index(current)

                    neighbors = tree.query_ball_point(current, r=0.2)
                    for neighbor_idx in neighbors:
                        if neighbor_idx not in visited:
                            visited.add(neighbor_idx)
                            queue.append(points_list[neighbor_idx])

                if len(component) > len(largest_component):
                    largest_component = component

        return largest_component


    def find_largest_boundary(self, perimeter_points):
        """
        Find the largest connected component of perimeter points.
        Assumes the room boundary is the largest component.
        """
        if not perimeter_points:
            return set()

        points_list = list(perimeter_points)
        tree = KDTree(points_list)
        visited = set()
        largest_component = set()

        for i, point in enumerate(points_list):
            if i not in visited:
                # BFS to find connected component
                component = set()
                queue = deque([point])
                visited.add(i)

                while queue:
                    current = queue.popleft()
                    component.add(current)
                    current_idx = points_list.index(current)

                    # Find neighbors within 0.2m (adjust based on grid resolution)
                    neighbors = tree.query_ball_point(current, r=0.2)
                    for neighbor_idx in neighbors:
                        if neighbor_idx not in visited:
                            visited.add(neighbor_idx)
                            queue.append(points_list[neighbor_idx])

                # Keep the largest component
                if len(component) > len(largest_component):
                    largest_component = component

        return largest_component


    def filter_perimeter_by_room(self, perimeter_points, room_polygon, margin=0.1):
        """
        Keep only perimeter points that are near the room's actual boundary.
        Uses the room polygon to filter out internal obstacles.
        """
        polygon = Polygon(room_polygon)
        boundary_points = set()

        for point in perimeter_points:
            # Check if point is on or very close to the polygon boundary
            shapely_point = Point(point[0], point[1])
            distance = shapely_point.distance(polygon.boundary)

            if distance <= margin:  # Within margin of room boundary
                boundary_points.add(point)

        return boundary_points

    def get_room_perimeter_points_2nd_pass(self, boundary_points, na):
        '''
        Second pass for room boundary points. The idea is that we take any point in the boundary points from
        get_room_perimeter_points_1st_pass(...) and start walking in all directions. If we reach another point
        from the original boundary points set, then it's the point that belongs to the same boundary, so take it
        out from the starting set and keep going. Once there are no points left to reach, we have established one
        boundary. Now take any point from the remaining set and repeat the same procedure. In the end we should
        have a set of boundaries within the room. Now we form a polygon from each of those sets and check which
        one is within another one. The outer polygon is what we want.

        :param boundary_points:
        :param na:
        :return:
        '''

        boundary_points = boundary_points.copy()
        all_sub_boundaries = list()
        current_sub_boundary = list()
        neighbours_found = 0
        crossroads = deque() # stack for crossroad points
        cbp = None
        cbp_prev = None
        cbp_next = None
        discovered_vectors = list()
        seen_4_cross = False

        #cbp = boundary_points.pop() # take any point as a starter
        # cbp is "current boundary point"
        for cbp in boundary_points:
            break

        while cbp:
            # add current point to the current boundary
            current_sub_boundary.append(cbp)
            neighbours_found = 0
            # now move forward until we see either a visited point or a crossroads (more than 2 valid paths from here)
            for move in na.MOVE_MOVES: # walk in all directions from current point until we find another point from the boundary or exhaust all moves
                new_x, new_y = na.apply(cbp[0], cbp[1], move)
                # count how many other boundary points we can see from this one
                if (new_x, new_y) in boundary_points and cbp_prev != (new_x, new_y) and not (((new_x, new_y), cbp) in discovered_vectors):
                    neighbours_found += 1
                    if neighbours_found == 1:
                        # The first neighbour that we find will be the regular one to explore
                        cbp_next = (new_x, new_y)
                    else:
                        # if there are more, then store them as directions in crossroads
                        discovered_vectors.append(((new_x, new_y), cbp))
                        crossroads.append(((new_x, new_y), cbp, current_sub_boundary.copy(), discovered_vectors))
                        #print("len(crossroads): ", len(crossroads), "cbp: ", cbp, "cbp_prev: ", cbp_prev, "(new_x, new_y): ", (new_x, new_y))


            # If we have 1 neighbour, then cbp is an end part of an unconnected boundary. We're not interested int this kind of path,
            # purge it.
            if neighbours_found < 1:
                cbp = None
                if len(crossroads) > 0:
                    cbp, cbp_prev, current_sub_boundary, discovered_vectors = crossroads.pop()
            else:
                if neighbours_found >= 3:
                    print("neighbours_found: ", neighbours_found, "len(crossroads): ", len(crossroads), "cbp: ", cbp, "cbp_prev: ", cbp_prev, " cbp_next: ", cbp_next)
                    #print("crossroads: ", crossroads)

                    for cr in [crossroads[i] for i in range(-2, 0)]:
                        print("3neighbour cr: ", cr)
                    seen_4_cross = True
                    #breakpoint()
                if neighbours_found == 2 and seen_4_cross:
                    #breakpoint()
                    pass
                cbp_prev = cbp
                cbp = cbp_next
                # see if we've found a closure for the current boundary
                if cbp in current_sub_boundary:
                    # if we see cbp already in the current path, then we have completed a loop and current_sub_boundary is a complete sub-boundary
                    cbp_ndx = current_sub_boundary.index(cbp)
                    current_sub_boundary = current_sub_boundary[cbp_ndx:]
                    all_sub_boundaries.append(current_sub_boundary)

                    # if there are more crossroads left, then explore those
                    cbp = None
                    if len(crossroads) > 0:
                        cbp, cbp_prev, current_sub_boundary, discovered_vectors = crossroads.pop()

        all_sub_boundaries = [sb for sb in all_sub_boundaries if len(sb) > 2]
        all_sub_boundaries = sorted(all_sub_boundaries, key=lambda boundary: Polygon(boundary).area)
        # for sb in all_sub_boundaries:
        #     print("SB: ", sb)
        #print("len(crossroads) at END: ", len(crossroads))
        return all_sub_boundaries

    def filter_double_boundaries(self, boundary_points, step=0.125):
        l_boundary_points = list(boundary_points)
        #l_boundary_points_sorted_by_x = sorted(l_boundary_points, key=lambda x: x[0])
        x_vals = [p[0] for p in l_boundary_points]
        x_vals_uq = list(set(x_vals))
        x_vals_uq = sorted(x_vals_uq, key=lambda x: x)
        boundary_points_2d = []
        # stack all points by their x-value, e.g.:
        #_______0______________1______________2______
        # (5.25, 6.88) | (5.37, 3.50) | (5.88, 3.50)
        # (5.25, 6.75) | (5.37, 3.62) | (5.88, 3.62)
        # (5.25, 6.62) |              | (5.88, 3.75)
        #
        for i in range(len(x_vals_uq)):
            new_col = [bp for bp in boundary_points if bp[0] == x_vals_uq[i]]
            new_col = sorted(new_col, key = lambda x: x[1])
            print(new_col)
            boundary_points_2d.append(new_col)

        all_removed = []
        # now if the difference between x values in two neighbouring columns is step (0.125) or thereabouts (due to
        # rounding issues), then look at y-values. if we have two or more y-values matching, then only keep 1,
        # ideally the longest series.
        for i in range(len(x_vals_uq) - 1):
            cur_col = boundary_points_2d[i]
            next_col = boundary_points_2d[i + 1]

            # cur_col = sorted(cur_col, key = lambda x: x[1])
            # next_col = sorted(next_col, key=lambda x: x[1])

            # if two columns are x-neighbours, then inspect their y-values
            if len(cur_col) > 0 and len(next_col) > 0 and next_col[0][0] - cur_col[0][0] < 1.5 * step:
                #print("X neighbours")
                removed_from_next_col = []
                new_next_col = next_col.copy()
                # get each y-value from cur_col and compare to all valuse in next_col
                for cc in range(len(cur_col) - 1):
                    #cur_y_val = cur_col[cc][1]
                    for nc in range (len(next_col) - 1):
                        #print(cur_col[cc][1], "==", next_col[nc][1], " and ", cur_col[cc + 1][1], " == ", next_col[nc + 1][1], " R: ", (cur_col[cc][1] == next_col[nc][1] and cur_col[cc + 1][1] == next_col[nc + 1][1]))
                        if cur_col[cc][1] == next_col[nc][1] and cur_col[cc + 1][1] == next_col[nc + 1][1] and cur_col[cc + 1][1] - cur_col[cc][1] < 1.5 * step:
                            # if we find a square of points, then eliminate these valuse in one of the columns
                            removed_from_next_col.append(next_col[nc])
                            removed_from_next_col.append(next_col[nc + 1])
                            #new_next_col[nc] = (-1, -1)
                            #new_next_col[nc + 1] = (-1, -1)
                            print("Removed ", next_col[nc])
                        else:
                            # if not, then keep the existing value
                            new_next_col[nc] = next_col[nc]

                all_removed.extend(removed_from_next_col)
                #boundary_points_2d[i + 1] = new_next_col
                #print("new_next_col: ", new_next_col)
            else:
                continue

        print("All removed: ", all_removed)
        print("new boundary points: ")
        for bp in boundary_points_2d:
            print(bp)

        all_vals = []
        for i in range(len(boundary_points_2d)):
            all_vals.extend(boundary_points_2d[i])

        #return set(all_vals)
        return boundary_points - set(all_removed)

    def visualize(self, collection_to_visualize):
        # Plot the room boundary and internal obstacles
        plt.figure(figsize=(8, 6))

        # Plot all perimeter points (including obstacles)
        all_perimeter = list(collection_to_visualize)
        plt.scatter([p[0] for p in all_perimeter], [p[1] for p in all_perimeter],
                    c='gray', s=5, alpha=0.5, label='All perimeters')

        # Plot filtered room boundary
        # room_boundary = extract_room_boundary(perimeter_points, room_polygon)
        # if room_boundary:
        #     boundary_list = list(room_boundary)
        #     plt.scatter([p[0] for p in boundary_list], [p[1] for p in boundary_list],
        #                 c='blue', s=10, label='Room boundary')

        plt.legend()
        plt.axis('equal')
        plt.title('Room Boundary vs Internal Obstacle Perimeters')
        plt.show()

if __name__ == "__main__":
    bc = BoundaryCalculations()
    boundary_points = {(8.62, 3.75), (7.62, 9.25), (6.75, 6.75), (5.5, 6.5), (5.5, 6.25), (5.5, 6.75), (5.5, 6.0), (8.0, 9.25), (5.5, 4.38), (5.5, 4.88), (7.88, 3.5), (7.12, 3.5), (7.25, 8.75), (5.88, 3.75), (7.5, 6.75), (6.38, 5.0), (5.62, 8.88), (8.62, 8.12), (8.62, 8.62), (8.25, 9.62), (6.25, 3.75), (6.38, 3.75), (8.62, 3.88), (6.75, 8.75), (8.62, 8.75), (8.25, 9.5), (8.25, 9.75), (8.12, 9.38), (8.5, 3.62), (5.5, 4.12), (5.5, 4.62), (8.5, 10.12), (5.5, 6.38), (8.25, 9.88), (8.5, 5.38), (8.5, 5.88), (5.5, 8.88), (7.88, 5.0), (5.75, 9.25), (7.75, 6.75), (5.38, 9.62), (7.0, 3.75), (6.5, 3.75), (8.62, 10.12), (6.88, 6.75), (6.25, 5.0), (8.62, 5.38), (8.62, 5.88), (6.12, 6.75), (6.62, 7.12), (5.5, 6.12), (5.5, 6.62), (8.5, 5.62), (7.25, 3.5), (5.62, 3.75), (7.75, 8.0), (7.12, 3.62), (7.75, 8.25), (6.0, 6.75), (7.62, 6.75), (7.75, 8.38), (6.38, 9.25), (6.5, 5.0), (6.12, 8.12), (8.62, 5.12), (6.38, 7.38), (8.62, 5.62), (6.88, 8.75), (6.25, 7.5), (8.62, 7.38), (8.62, 7.88), (6.12, 8.0), (6.12, 8.25), (8.5, 5.25), (7.25, 7.12), (7.38, 4.75), (8.5, 5.5), (8.5, 5.75), (7.88, 9.25), (5.88, 5.12), (7.62, 8.5), (6.5, 7.25), (8.12, 6.75), (6.0, 8.38), (8.62, 7.12), (8.62, 7.62), (6.25, 9.25), (8.25, 6.75), (8.62, 5.0), (8.62, 5.25), (8.62, 5.5), (8.62, 5.75), (6.62, 4.88), (5.5, 3.88), (6.62, 9.25), (7.38, 6.75), (5.75, 6.0), (5.75, 6.25), (5.75, 6.5), (5.25, 10.0), (7.12, 7.12), (7.5, 8.62), (7.0, 9.25), (8.38, 6.75), (6.5, 9.25), (6.88, 3.75), (6.12, 3.75), (8.62, 7.0), (8.62, 7.25), (8.62, 7.5), (8.62, 7.75), (5.5, 10.12), (5.5, 5.38), (5.5, 5.88), (5.5, 10.0), (7.38, 8.75), (7.25, 9.25), (5.75, 6.38), (5.75, 6.88), (5.75, 8.88), (7.0, 7.12), (7.62, 3.5), (8.62, 9.12), (8.62, 9.62), (8.62, 4.38), (8.62, 4.88), (8.62, 9.75), (6.75, 3.75), (5.5, 5.12), (5.5, 5.62), (8.62, 9.0), (8.62, 9.25), (8.62, 9.5), (5.5, 3.75), (8.62, 9.38), (8.62, 9.88), (8.0, 6.75), (5.75, 6.12), (5.75, 6.62), (8.38, 10.12), (6.0, 8.12), (6.0, 8.62), (7.62, 5.0), (5.38, 10.0), (7.12, 9.25), (7.5, 3.5), (6.0, 8.0), (6.0, 8.25), (6.0, 8.5), (5.88, 9.25), (6.25, 6.75), (6.75, 7.12), (6.62, 6.75), (5.5, 5.0), (5.5, 5.25), (5.5, 5.5), (7.38, 3.5), (5.5, 5.75), (7.12, 4.62), (7.75, 7.88), (8.38, 3.5), (7.75, 3.5), (6.5, 6.75), (5.88, 6.88), (6.88, 9.25), (8.62, 4.75), (6.62, 8.62), (8.62, 4.0), (8.62, 4.25), (8.62, 4.5), (6.25, 8.38), (6.12, 7.88), (5.25, 9.62), (8.5, 6.0), (8.5, 6.25), (8.5, 6.5), (5.25, 9.75), (7.25, 6.75), (7.75, 7.62), (7.0, 4.62), (7.75, 5.0), (5.25, 9.88), (6.0, 7.38), (6.0, 7.88), (6.0, 3.75), (6.5, 8.5), (6.38, 6.75), (6.88, 7.12), (8.62, 6.0), (8.62, 6.25), (8.62, 6.5), (8.62, 6.75), (5.62, 9.38), (8.25, 10.12), (6.75, 9.25), (5.5, 9.12), (5.5, 9.62), (6.12, 5.0), (8.25, 10.0), (6.12, 7.62), (8.0, 3.5), (5.5, 9.0), (5.5, 9.25), (5.5, 9.5), (5.5, 9.75), (7.38, 7.25), (8.5, 6.38), (7.25, 4.62), (5.5, 9.38), (5.5, 9.88), (5.75, 5.38), (5.75, 5.88), (7.88, 6.75), (6.0, 7.12), (6.0, 7.62), (7.12, 6.75), (7.75, 7.75), (6.0, 5.0), (6.88, 4.75), (7.5, 9.25), (5.88, 6.75), (8.12, 3.5), (7.5, 7.38), (8.62, 4.12), (8.62, 4.62), (8.62, 8.0), (8.62, 8.25), (8.62, 8.5), (6.38, 8.38), (6.62, 3.75), (8.62, 6.88), (8.62, 6.38), (8.25, 3.5), (8.62, 8.38), (8.62, 8.88), (6.12, 7.75), (8.0, 5.0), (7.88, 8.12), (8.5, 6.12), (8.5, 6.62), (5.75, 5.62), (7.38, 9.25), (5.75, 3.75), (7.75, 9.25), (8.38, 5.12), (7.12, 8.75), (6.0, 7.5), (6.0, 7.75), (7.0, 6.75), (6.0, 7.0), (5.88, 8.75), (6.0, 7.25), (7.62, 7.5), (8.12, 5.0), (8.62, 10.0), (6.75, 4.75), (8.62, 6.12), (8.62, 6.62), (8.25, 5.0), (6.62, 5.0), (5.5, 4.0), (6.12, 9.25), (5.5, 4.25), (5.5, 4.5), (5.5, 4.75), (5.75, 5.25), (5.75, 5.5), (5.75, 5.75), (5.62, 6.75), (7.5, 4.75), (6.0, 9.25), (7.62, 4.88), (7.0, 8.75)}
    de_b = bc.filter_double_boundaries(boundary_points)
    bc.visualize(de_b)
