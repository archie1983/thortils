# Keyboard control of Ai2Thor

import thortils
import thortils.constants as constants
from thortils.utils import getch
import argparse
import time, cv2, os

import prior

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from ai2_thor_model_training.ae_utils import (NavigationUtils, action_mapping,
                                                              action_to_index, index_to_action, inverted_action_mapping,
                                                              AI2THORUtils, get_path_length, get_centre_of_the_room,
                                                              room_this_point_belongs_to, get_rooms_ground_truth)

from thortils.agent import thor_reachable_positions, thor_agent_position, thor_agent_pose
from thortils.utils import roundany, PriorityQueue, normalize_angles, euclidean_dist

import numpy as np

#point = Point(0.5, 0.5)
#polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
#print(polygon.contains(point))

def is_point_inside_room(point_to_test, room_polygon):
    (x, y, z) = point_to_test
    point = Point(x, z)
    polygon = Polygon(room_polygon)
    return polygon.contains(point)

def what_room_is_point_in(rooms, point):
    for room in rooms:
        if is_point_inside_room(point, room[1]):
            return room[0]
    return "NONE"

def get_rooms(house):
    rooms = []
    for room in house["rooms"]:
        room_poly = [(corner["x"], corner["z"]) for corner in room["floorPolygon"]]
        #print(room["roomType"] + " # " + str(room["floorPolygon"]))
        #print(room["roomType"] + " ?? " + str(room_poly))
        rooms.append((room["roomType"], room_poly))

    #print(rooms)

    #for room in rooms:
    #    print(room[0])
    return rooms

def get_visible_object_names(event):
    vis_objs = []
    objs = thortils.thor_visible_objects(event)

    for obj in objs:
        vis_objs.append(obj['objectType'])

    return vis_objs

def print_controls(controls):
    reverse = {controls[k]:k for k in controls}
    ss =f"""
            {reverse['MoveAhead']}
        (MoveAhead)

    {reverse['RotateLeft']}                 {reverse['RotateRight']}
(RotateLeft)     (RotateRight)

    {reverse['LookUp']}
(LookUp)

    {reverse['LookDown']}
(LookDown)

    q
(quit)
    """
    print(ss)

cnt = 0
target_dir = "scene_pics"

def store_frame(event):
    global cnt
    #print(len(event.third_party_camera_frames[0]))
    #print(event.third_party_camera_frames[0])
    img = event.cv2img
    #dst = cv2.resize(img, (target_size, target_size), interpolation=cv2.INTER_LANCZOS4)

    #object_type = object_id.split("|")[0].lower()
    #target_dir = os.path.join("images", scene_name, object_type)
    #h = hashlib.md5()
    #h.update(json.dumps(point, sort_keys=True).encode("utf8"))
    #h.update(json.dumps(v, sort_keys=True).encode("utf8"))

    cnt+=1
    os.makedirs(target_dir, exist_ok=True)
    cv2.imwrite(os.path.join(target_dir, str(cnt) + ".png"), img)

def get_agent_pos_and_rotation(controller):
    pos = (controller.last_event.metadata["agent"]["position"]["x"], controller.last_event.metadata["agent"]["position"]["y"], controller.last_event.metadata["agent"]["position"]["z"])
    rtn = (controller.last_event.metadata["agent"]["rotation"]["x"], controller.last_event.metadata["agent"]["rotation"]["y"], controller.last_event.metadata["agent"]["rotation"]["z"])
    return (pos, rtn)

##
# From the max and min of the habitat coordinates we can generate full grid of the habitat.
# Later we can infer unreachable positions from this and reachable positions.
##
def create_full_grid_from_room_layout(rooms_in_habitat, step = 0.25):
    #print("AE, rooms_in_habitat: ", rooms_in_habitat)
    room_coords = [] # here we will store coordinates of every corner of each room
    [room_coords.extend(r[1]) for r in rooms_in_habitat]
    zf = lambda x: zip(*x) # this will allow to turn the tuples of coordinates into two lists - X and Y coordinates lists.
    [x_coords, y_coords] = zf(room_coords) # get the two lists
    # get the max and min coordinates from each list
    min_x = min(x_coords)
    min_y = min(y_coords)
    max_x = max(x_coords)
    max_y = max(y_coords)

    # Create the grid coordinates
    x_coords = np.arange(min_x, max_x + step, step)
    y_coords = np.arange(min_y, max_y + step, step)

    # Create meshgrid
    X, Y = np.meshgrid(x_coords, y_coords)

    # Create list of (x, y) tuples
    all_positions = list(zip(X.flatten(), Y.flatten()))
    return all_positions

def add_buffer_to_unreachable(reachable_points, all_grid_points, step=0.25, buffer_size=1):
    """
    Add buffer around unreachable positions using grid-based approach.

    Parameters:
    reachable_positions: list of (x, y) tuples from AI2-THOR
    all_grid_points: full list of all (x, y) tuples including both reachable and unreachable
    step: grid step size
    buffer_size: number of grid cells to buffer (default: 1 cell = 0.25m)
    """

    # Find unreachable positions
    unreachable = all_grid_points - reachable_points

    # Add buffer around unreachable positions
    buffered_unreachable = set(unreachable)  # Start with original unreachable

    # Define neighbor directions (4-connected or 8-connected)
    directions_4 = [(0, step), (0, -step), (step, 0), (-step, 0)]
    directions_8 = directions_4 + [(step, step), (step, -step), (-step, step), (-step, -step)]

    # Add buffer layers
    for _ in range(buffer_size):
        new_buffer = set()
        for point in buffered_unreachable:
            x, z = point
            for dx, dz in directions_8:  # Use 8-connected for better coverage
                neighbor = (round(x + dx, 2), round(z + dz, 2))
                if neighbor in all_grid_points:
                    new_buffer.add(neighbor)
        buffered_unreachable.update(new_buffer)

    # Final safe positions are all grid points minus buffered unreachable
    safe_positions = all_grid_points - buffered_unreachable

    return safe_positions, buffered_unreachable

def main(init_func=None, step_func=None):
    parser = argparse.ArgumentParser(
        description="Keyboard control of agent in ai2thor")
    parser.add_argument("-s", "--scene",
                        type=str, help="scene. E.g. FloorPlan1",
                        default="FloorPlan1")
    args = parser.parse_args()

    controls = {
        "w": "MoveAhead",
        "a": "RotateLeft",
        "d": "RotateRight",
        "e": "LookUp",
        "c": "LookDown"
    }
    print_controls(controls)

    dataset = prior.load_dataset("procthor-10k")
    house = dataset["train"][43] # 10
    #house = dataset["train"][88]
    #print(house)
    args.scene = house

    rooms = get_rooms(house)

    #controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})
    # GRID_SIZE can be e.g. 0.25, 0.125, 0.1, 0.3. But if we have 0.2 or 0.15, then AI2-Thor returns
    # insane grid locations (e.g. with 0.15 we get (0.39999961853027344, 5.75), which shouldn't be possible).
    # I'm not sure why this happens.
    controller = thortils.launch_controller({"scene": args.scene, "VISIBILITY_DISTANCE": 3.0, "GRID_SIZE": 0.125})
    grid_size = controller.initialization_parameters["gridSize"]

    # AE: Required infrastructure for calculating path lengths
    nu = NavigationUtils(step = grid_size)
    atu = AI2THORUtils()
    atu.set_controller(controller)

    reachable_positions = [
        tuple(map(lambda x: round(roundany(x, grid_size), 2), pos))
        for pos in thor_reachable_positions(controller)]

    #reachable_positions = [
    #    tuple(map(lambda x: round(x, 2), pos))
    #    for pos in thor_reachable_positions(controller)]

    #event = controller.step(action="GetReachablePositions")
    #r_positions = event.metadata["actionReturn"]
    #r_positions = [(pos['x'], pos['z']) for pos in r_positions]

    rooms_in_habitat = get_rooms_ground_truth(house)
    #print("reachable_positions: ", reachable_positions)
    # AE: Path length infra set up
    #pos_ba = thor_reachable_positions(controller, by_axes = True)
    #print("AE, by axes: ", pos_ba)
    full_grid = create_full_grid_from_room_layout(rooms_in_habitat, step=grid_size)
    full_grid = [tuple(map(lambda x: round(x, 2), pos)) for pos in full_grid]
    unreachable_postions = set(full_grid) - set(reachable_positions)
    (safe_pos, buf_unreachable_pos) = add_buffer_to_unreachable(set(reachable_positions), set(full_grid), step=grid_size)
    #print("unreachable_postions: ", unreachable_postions)
    #print("reachable_positions: ", r_positions) #reachable_positions

    event = controller.step(
        action="AddThirdPartyCamera",
        position=dict(x=-4.25, y=2, z=-2.5),
        rotation=dict(x=90, y=0, z=0),
        fieldOfView=120
    )

    if init_func is not None:
        config = init_func(controller)

    while True:
        k = getch()
        if k == "q":
            print("bye.")
            break

        if k in controls:
            action = controls[k]
            params = constants.MOVEMENT_PARAMS[action]

            (p, r) = thortils.thor_agent_pose(controller, as_tuple=True)
            c_yaw = int(r[1])
            if action == "MoveAhead":
                if c_yaw in [45, 135, 225, 315]:
                    params["moveMagnitude"] = (grid_size**2*2)**0.5 # Pythagorean theorem c = sqrt(a^2 + a^2) #0.353553391
                else:
                    params["moveMagnitude"] = grid_size #0.25

            print("MOVE PARAMS: ", params)
            event = controller.step(action=action, **params)
            event = controller.step(action="Pass")
            if step_func is not None:
                step_func(event, config)

            pose = thortils.thor_agent_pose(controller, as_tuple=True)

            #print(pose)
            (p, r) = pose
            objs = get_visible_object_names(event)

            store_frame(event)

            #print("{} | Agent pose: {}".format(k, pose) + " Room: " + what_room_is_point_in(rooms, p) + " ## " + str(objs))
            print("{} | Agent pose: {}".format(k, pose))

            # AE: Now that we have a pose, let's calculate how far is it to the centre of the room
            point_for_room_search = (p[0], "", p[2])
            #print("AE: ", rooms_in_habitat, " :: ", point_for_room_search)
            room_of_placement = room_this_point_belongs_to(rooms_in_habitat, point_for_room_search)
            # #print("AE: room_of_placement: ", room_of_placement)
            # print("AE: rooms_in_habitat: ", rooms_in_habitat)
            # room_centre = room_of_placement[2]
            # try:
            #     path_length = nu.get_path_cost_to_target_point(pose,
            #                                                    room_centre,
            #                                                    reachable_positions)
            # except ValueError:
            #     path_length = 0
            #     print("AE: No Path Found")

            cur_pos = get_agent_pos_and_rotation(controller)
            place_with_rtn = (cur_pos[0][0], cur_pos[0][2], cur_pos[1][1])

            try:
                # TODO: Ignore doors that are very close by (e.g. right behind us)
                current_target_point = nu.find_door_target(place_with_rtn,
                                                                     rooms_in_habitat,
                                                                     reachable_positions,
                                                                     controller, close_enough=0.25, step=grid_size)

                t1 = time.time()
                path_length = nu.get_path_cost_to_target_point(pose,
                                                               current_target_point,
                                                               reachable_positions, close_enough=0.25, step=grid_size)
                print("AE: path plan time: ", (time.time() - t1))
            except ValueError as e:
                path_length = 0
                print("AE: No Path Found", e)

            print("AE: Path Length: ", path_length)
            (cur_path, reachable_positions, start, dest) = nu.get_last_path_and_params()
            print("AE: Path: ", cur_path)
            atu.visualise_path2(cur_path, reachable_positions, unreachable_postions, rooms_in_habitat, start, dest,
                                show_unreachable_pos = True,
                                show_reachable_pos = False)
            #atu.visualise_path2(cur_path, reachable_positions, buf_unreachable_pos, rooms_in_habitat, start, dest, show_unreachable_pos=True)

if __name__ == "__main__":
    main()
