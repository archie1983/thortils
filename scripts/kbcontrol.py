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
                                                              room_this_point_belongs_to, get_rooms_ground_truth,
                                                              create_full_grid_from_room_layout, add_buffer_to_unreachable)

from ai2_thor_model_training.training_data_extraction import RobotNavigationControl

from thortils.agent import thor_reachable_positions, thor_agent_position, thor_agent_pose
from thortils.utils import roundany, PriorityQueue, normalize_angles, euclidean_dist

import numpy as np

from ae_path_compare import PathCompareClient

#point = Point(0.5, 0.5)
#polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
#print(polygon.contains(point))

class DistanceReductionReward:
    def __init__(self, scale=1.0):
        self.scale = scale
        self.prev_distance = None
        self.best_distance_so_far = None

    def __call__(self, obs, extra_obs, action):
        #print("D1")
        reward = 0.0
        #distance_left = obs['distance_left']
        distance_left = extra_obs['distanceleft']

        if obs['is_first']:
            self.best_distance_so_far = distance_left
        else:
            if self.best_distance_so_far > distance_left:
                '''
                if we improved best distance, then reward is the improvement factor
                '''
                #reward = self.scale * (self.best_distance_so_far - distance_left)
                reward = 1
                self.best_distance_so_far = distance_left
                #print("BIG REW: ", reward)
                #print("r", reward, end="", sep="")
                print("r", end="", sep="")
            # elif self.best_distance_so_far == distance_left:
            #     '''
            #     if no improvement, then bigger penalty. No movement needs to be discouraged
            #     '''
            #     reward = -0.25
            elif self.best_distance_so_far < distance_left and self.prev_distance < distance_left:
                '''
                if we have moved away from the target, then penalty by the reduction
                '''
                #reward = self.scale * (self.prev_distance - distance_left)
                reward = -0.5
            elif self.best_distance_so_far <= distance_left and self.prev_distance > distance_left:
                '''
                if we have improved our position from last time, but not yet the best path, then small reward
                '''
                reward = 0.25
            elif self.best_distance_so_far <= distance_left and self.prev_distance == distance_left:
                '''
                if no improvement since last time, then penalty to discourage not moving
                '''
                reward = -0.25
            else:
                '''
                shouldn't happen. If it does, then the above code has error.
                '''
                print("CHECK DistanceReductionReward CODE!!!")
                exit()

        self.prev_distance = distance_left
        #print("D2")

        return np.float32(reward)

class StepCountPenalizer:
    def __init__(self, scale=1.0):
        self.scale = scale
        self.steps_done = 0

    def __call__(self, obs, extra_obs, action):
        reward = 0.0
        if obs['is_first']:
            self.steps_done = 0
        else:
            self.steps_done += 1

        if extra_obs['all_target_dists_initial'] is not None and len(extra_obs['all_target_dists_initial']) > 0 and self.steps_done > 4 * np.max(extra_obs['all_target_dists_initial']):
            reward = -0.25
        #if self.steps_done > extra_obs['initial_distance']:
        #    reward = -0.1

        return np.float32(reward * self.scale)

##
# Issue a reward for achieving the target - once per scene
##
class TargetAchievedRewardForDoor:
    def __init__(self, epsilon = 0.0, min_steps_in_new_room = 3, max_steps_in_new_room = 10):
        '''
        :param epsilon: How close is close enough to issue the reward
        '''
        self.reward_issued = False
        self.min_steps_in_new_room = min_steps_in_new_room
        self.max_steps_in_new_room = max_steps_in_new_room
        self.epsilon = epsilon
        self.steps_done = 0

    def __call__(self, obs, extra_obs, stop_action = False):
        #print("T1")
        reward = 0
        if obs['is_first']:
            self.reward_issued = False
            self.steps_done = 0
        elif not self.reward_issued and stop_action:
            '''
            We only want to issue this reward once the STOP action has been issued by the model. And at that point we will calculate
            how much we award based on what has been achieved.
            '''
            # high reward for achieving epsilon requirement for any door
            for dist in extra_obs['all_target_dists']:
                if dist <= self.epsilon:
                    reward += 100
                    break

            if (extra_obs['stepsafterroomchange'] > 0):
                reward += 100

            # high reward for correct amount of steps in the new room
            if (extra_obs['stepsafterroomchange'] <= self.max_steps_in_new_room and extra_obs['stepsafterroomchange'] >= self.min_steps_in_new_room):
                reward += 100

            # Participation prize if any of the distances have become smaller
            # Participation prize equals to the best reduction of the distances
            if extra_obs['all_target_dists_initial'] is not None and len(extra_obs['all_target_dists']) == len(extra_obs['all_target_dists_initial']) and len(extra_obs['all_target_dists']) > 0:
                reward += 2 * max([d1 - d2 for d1, d2 in zip(extra_obs['all_target_dists_initial'], extra_obs['all_target_dists'])])

            # If none of the above rewards have been earned, then check if it needs a penalty for
            # early STOP (not walking enough to get even through the nearest door)
            if reward <= 0:
                if extra_obs['all_target_dists_initial'] is not None and len(extra_obs['all_target_dists_initial']) > 0:
                    min_distance_walk = np.min(extra_obs['all_target_dists_initial'])
                    mean_distance_walk = np.mean(extra_obs['all_target_dists_initial'])
                else:
                    min_distance_walk = extra_obs['initial_distance']
                    mean_distance_walk = min_distance_walk
                if self.steps_done < mean_distance_walk:
                    reward = -1 * (min_distance_walk - self.steps_done)

                reward = max(reward, -100)

            self.reward_issued = True
            #print("final reward: ", reward, " = ", extra_obs['initial_distance'], " - ", extra_obs['distanceleft'])
        else:
            self.steps_done += 1
        return np.float32(reward)

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

    {reverse['Teleport']}
(Teleport to defined place)

    1
(take sequence pics)

    2
(send sequence pics)

    3
(compare sequence pics)

    s
(Simulate STOP)

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

    return img

def get_agent_pos_and_rotation(controller):
    pos = (controller.last_event.metadata["agent"]["position"]["x"], controller.last_event.metadata["agent"]["position"]["y"], controller.last_event.metadata["agent"]["position"]["z"])
    rtn = (controller.last_event.metadata["agent"]["rotation"]["x"], controller.last_event.metadata["agent"]["rotation"]["y"], controller.last_event.metadata["agent"]["rotation"]["z"])
    return (pos, rtn)

def euclidean_dist_to_all_targets(all_door_targets, cur_pos):
    dists = []
    if all_door_targets != None:
        for dt in all_door_targets:
            p1 = (dt['pos']['x'], dt['pos']['z'])
            p2 = cur_pos
            dists.append(euclidean_dist(p1, p2))
    return dists

def main(init_func=None, step_func=None):
    USE_RNC = True
    if USE_RNC:
        rnc = RobotNavigationControl()
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
        "c": "LookDown",
        "t": "Teleport"
    }
    print_controls(controls)

    dataset = prior.load_dataset("procthor-10k")
    #house = dataset["train"][43] # 10
    #house = dataset["train"][88]
    #house = dataset["test"][658]
    #house = dataset["test"][709]
    house = dataset["test"][858]
    #print(house)
    args.scene = house

    rooms = get_rooms(house)

    #controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})
    # GRID_SIZE can be e.g. 0.25, 0.125, 0.1, 0.3. But if we have 0.2 or 0.15, then AI2-Thor returns
    # insane grid locations (e.g. with 0.15 we get (0.39999961853027344, 5.75), which shouldn't be possible).
    # I'm not sure why this happens.
    controller = thortils.launch_controller({"scene": args.scene,
                                             "VISIBILITY_DISTANCE": 3.0,
                                             "IMAGE_WIDTH": 600,
                                             "IMAGE_HEIGHT": 600,
                                             "GRID_SIZE": 0.125,
                                             "headless": False,
                                             "quality": 'Low'})
    grid_size = controller.initialization_parameters["gridSize"]

    # AE: Required infrastructure for calculating path lengths
    nu = NavigationUtils(step = grid_size)
    atu = AI2THORUtils()
    agent = PathCompareClient(jetson_ip="192.168.0.109", port=5555)
    atu.set_controller(controller)
    if USE_RNC:
        rnc.set_controller(controller)

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
    #print(house["rooms"])
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

    reward_close_enough = 0.25
    rewards = [
        StepCountPenalizer(scale=1.0),
        DistanceReductionReward(scale=1.0),
        TargetAchievedRewardForDoor(epsilon=reward_close_enough)
    ]
    is_first = True
    current_target_point = None
    store_ref_path = False
    path_id = 0

    steps_in_new_room = 0
    cur_pos = rnc.get_agent_pos_and_rotation()
    start_pos = (cur_pos[0][0], cur_pos[0][2])
    starting_room = room_this_point_belongs_to(rooms_in_habitat, cur_pos[0])
    all_target_dists_initial = None
    all_target_dists = None
    is_stop_simul = False

    while True:
        k = getch()
        if k == "q":
            print("bye.")
            break
        elif k == "1":
            store_ref_path = True
            path_id += 1
            ref_path_batch = []
            print("Starting taking pics for a sequence")
        elif k == "2":
            print("Sending sequence pics")
            store_ref_path = False
            # Stack into single numpy array
            image_batch = np.stack(ref_path_batch, axis=0)
            print(agent.store_ref_path(image_batch, str(path_id)))
            # print(f"Batch shape: {image_batch.shape}")
            # print(f"Batch dtype: {image_batch.dtype}")
        elif k == "3":
            print("Comparing sequence pics")
            store_ref_path = False
            image_batch = np.stack(ref_path_batch, axis=0)
            print(agent.qry_path_similarity(image_batch))
        elif k == "s":
            print("simulating STOP")
            is_stop_simul = True

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

            if action == "Teleport":
                if USE_RNC:
                    # [1.0, 0.88, 5.75], [0.0, 180, 0.0]
                    #place_with_rtn = (1.0, 5.75, 180)
                    # [4.12, 0.88, 5.62], [0.0, 315, 0.0]
                    #place_with_rtn = (4.12, 5.62, 315)
                    #[[10.5, 0.88, 7.5], [0.0, 270, 0.0]]
                    place_with_rtn = (10.5, 7.5, 270)
                    rnc.teleport_to(place_with_rtn)
                else:
                    # [1.0, 0.88, 5.75], [0.0, 180, 0.0]
                    params["position"] = dict(x=1.00, y=0.9009997844696045, z=5.75)
                    #params["position"] = dict(x=7.0, y=0.9009997844696045, z=5.625)
                    params["rotation"] = dict(x=0.0, y=180, z=0.0)
                    # self.controller.step(action="Teleport", **pos_navigate_to)
                    event = controller.step(action=action, **params)
                    event = controller.step(action="Pass")
            else:
                #print("MOVE PARAMS: ", params)
                if USE_RNC:
                    #raw_action = index_to_action(int(action['action']))
                    rnc.execute_action(action, moveMagnitude=grid_size, grid_size=grid_size, adhere_to_grid=True)
                else:
                    event = controller.step(action=action, **params)

                event = controller.step(action="Pass")

                if step_func is not None:
                    step_func(event, config)

            pose = thortils.thor_agent_pose(controller, as_tuple=True)

            #print(pose)
            (p, r) = pose
            objs = get_visible_object_names(event)

            cur_img = store_frame(event)
            if store_ref_path:
                ref_path_batch.append(cur_img)

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
                if current_target_point == None:
                    current_target_point, all_door_targets = nu.find_door_target(place_with_rtn,
                                                                                 rooms_in_habitat,
                                                                                 reachable_positions,
                                                                                 house,
                                                                                 controller, close_enough=0.25,
                                                                                 step=grid_size, extend_path=True)
                    #current_target_point = current_target_point[0]
                path_length = nu.get_path_cost_to_target_point(pose,
                                                               current_target_point,
                                                               reachable_positions, close_enough=0.25,
                                                               step=grid_size, debug=False)
                # print("AE: path plan time: ", (time.time() - t1))

                t1 = time.time()

                # if we've been successful so far, then we can now look up room type
                trg_pos_xy = (current_target_point.x, "", current_target_point.y)
                #self.target_room = room_this_point_belongs_to(self.rooms_in_habitat, trg_pos_xy)
            except ValueError as e:
                path_length = 0
                print("AE: No Path Found", e)

            #print("AE: Path Length: ", path_length)
            (cur_path, reachable_positions, start, dest) = nu.get_last_path_and_params()
            #print("AE: Path: ", cur_path)

            cur_pos_xy = (cur_pos[0][0], cur_pos[0][2])
            current_room = room_this_point_belongs_to(rooms_in_habitat, cur_pos[0])

            if starting_room is not None and current_room != starting_room:
                steps_in_new_room += 1
                if steps_in_new_room > 15:
                    #steps_in_new_room = 0
                    starting_room = current_room

            all_current_trg_dists = euclidean_dist_to_all_targets(all_door_targets, cur_pos_xy)
            if all_target_dists_initial == None:
                all_target_dists_initial = all_current_trg_dists

            obs = dict(is_first = is_first, distance_left = path_length)
            extra_obs = dict(distanceleft = path_length,
                             stepsafterroomchange = steps_in_new_room,
                             all_target_dists = all_current_trg_dists,
                             all_target_dists_initial = all_target_dists_initial)
            r = 0
            cr = 0
            for fn in rewards:
                cr = fn(obs, extra_obs, is_stop_simul)
                r += cr
                print("RT: ", fn.__class__, " : ", cr)
            print("REWARD at this steo: ", r, " extra_obs: ", extra_obs)

            # reward = sum([fn(obs, extra_obs, is_stop_simul) for fn in rewards])
            # print("REWARD at this step: ", reward, " extra_obs: ", extra_obs)

            is_first = False
            if is_stop_simul:
                is_stop_simul = False
                is_first = True

            all_visible_doors = nu.get_all_visible_doors(controller)
            print("DOORVIS: ", len(all_visible_doors))

            # Visualize path and obstructed space
            atu.visualise_path2(cur_path, reachable_positions, unreachable_postions, rooms_in_habitat, start, dest,
                                show_unreachable_pos = True,
                                show_reachable_pos = False)
            atu.visualise_path2(cur_path, reachable_positions, buf_unreachable_pos, rooms_in_habitat, start, dest, show_unreachable_pos=True)

if __name__ == "__main__":
    main()
