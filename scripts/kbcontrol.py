# Keyboard control of Ai2Thor

import thortils
import thortils.constants as constants
from thortils.utils import getch
import argparse
import time, cv2, os

import prior

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

point = Point(0.5, 0.5)
polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
print(polygon.contains(point))

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

    for room in rooms:
        print(room[0])
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
    house = dataset["train"][55]
    args.scene = house

    rooms = get_rooms(house)

    #controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})
    controller = thortils.launch_controller({"scene": args.scene, "VISIBILITY_DISTANCE": 3.0})

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
            event = controller.step(action=action, **params)
            event = controller.step(action="Pass")
            if step_func is not None:
                step_func(event, config)

            pose = thortils.thor_agent_pose(controller, as_tuple=True)
            #print(pose)
            (p, r) = pose
            objs = get_visible_object_names(event)

            store_frame(event)

            print("{} | Agent pose: {}".format(k, pose) + " Room: " + what_room_is_point_in(rooms, p) + " ## " + str(objs))

if __name__ == "__main__":
    main()
