import numpy as np
import colorsys
import matplotlib.pyplot as plt
import yaml
import time
import argparse

import pybullet as p
import pybullet_data


ROBOT_URDF_PATH = "/resources/robots/turtlebot/turtlebot.urdf"
PLANE_URDF_PATH = "/resources/object/plane/plane.urdf"


def create_obs_wall(x, y, demension):
    half_extents = demension  # Half of the width, height, and depth of the box
    # box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.5, 0.5, 0.5, 1])
        # Set the initial position and orientation of the box
    box_pos = [x, y, 0]  # Position (x, y, z)
    box_orn = p.getQuaternionFromEuler([0, 0, 0])  # Orientation in quaternion format (roll, pitch, yaw)
    p.createMultiBody(baseMass=1, baseVisualShapeIndex=visual_shape_id,
                                basePosition=box_pos, baseOrientation=box_orn)
    # p.createMultiBody(baseMass=1, baseCollisionShapeIndex=box_id, baseVisualShapeIndex=visual_shape_id,
    #                             basePosition=box_pos, baseOrientation=box_orn)

def create_env(map_yaml, schedule_yaml):
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.loadURDF(PLANE_URDF_PATH)

    with open(map_yaml) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(schedule_yaml) as states_file:
        schedule = yaml.load(states_file, Loader=yaml.FullLoader)

    xmin = 0.0
    xmax = map["map"]["dimensions"][0]
    ymin = 0.0
    ymax = map["map"]["dimensions"][1]
    half_wall_width = 0.05
    create_obs_wall(xmin-0.01-0.5, -0.5+ymax/2.0, [half_wall_width-0.01, ymax/2.0, 0.1])
    create_obs_wall(-0.5+xmax/2.0, ymin-0.01-0.5, [xmax/2.0, half_wall_width-0.01, 0.1])
    create_obs_wall(xmax+0.01-0.5, -0.5+ymax/2.0, [half_wall_width-0.01, ymax/2.0, 0.1])
    create_obs_wall(-0.5+xmax/2.0, ymax+0.01-0.5, [xmax/2.0, half_wall_width-0.01, 0.1])

    for obs in map["map"]["obstacles"]:
        x, y = obs[0], obs[1]
        create_obs_wall(x, y, [0.45, 0.45, 0.1])

    robots = []
    num_robots = len(map["agents"])
    colors = generate_colors(num_robots)
    schedule_new = []
    for i in range(num_robots):
        # copy data for i-th agent
        data = map["agents"][i]
        
        name = data["name"]
        schedule_i = []
        for stepi in schedule["schedule"][name]:
            schedule_i.append([stepi["x"], stepi["y"]])
        # print(name)
        # print(schedule["schedule"][name])
        schedule_new.append(schedule_i)
        # schedule_new.append(schedule["schedule"][name])

        r, g, b = colors[i]
        robot_id = p.loadURDF(ROBOT_URDF_PATH, basePosition=[data["start"][0], data["start"][1], 0.1])
        p.changeVisualShape(robot_id, 14, rgbaColor=[r, g, b, 1])  
        p.changeVisualShape(robot_id, 19, rgbaColor=[r, g, b, 1]) 
        # p.changeVisualShape(robot_id, 26, rgbaColor=[r, g, b, 1])  
        robots.append(robot_id)

        visualShapeId = p.createVisualShape(shapeType=p.GEOM_CYLINDER, radius=0.05, length=2, rgbaColor=[r, g, b, 0.5])
        boxPosition = [data["goal"][0], data["goal"][1], 0]  # Position (x, y, z)
        boxOrientation = p.getQuaternionFromEuler([0, 0, 0])  # Orientation in quaternion format (roll, pitch, yaw)
        p.createMultiBody(baseMass=1, baseVisualShapeIndex=visualShapeId,
                                basePosition=boxPosition, baseOrientation=boxOrientation)
        
    return robots, schedule_new

def generate_colors(n):
    colors = []
    for i in range(n):
        hue = i / n
        rgb = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        colors.append([int(c * 255) for c in rgb])
    return colors


# def visualize_colors(colors):
#     fig, ax = plt.subplots(1, 1, figsize=(8, 1))
#     for i, color in enumerate(colors):
#         rect = plt.Rectangle((i, 0), 1, 1, color=[c / 255.0 for c in color])
#         ax.add_patch(rect)
#     ax.set_xlim(0, len(colors))
#     ax.set_ylim(0, 1)
#     ax.axis('off')
#     plt.show()

def move_robot(robots, schedule):
    num_robots = len(robots)
    # max_length = max(len(sublist) for sublist in schedule)
    
    resolution = 10

    new_paths = extend_path(schedule, resolution)
    length_path = len(new_paths[0])
    print(new_paths[0])
    print()
    print(new_paths[1])
    for i in range(length_path-1):
        x = []
        y = []
        for j in range(num_robots):
            cur_data = new_paths[j][i]
            p.resetBasePositionAndOrientation(robots[j],posObj=[cur_data[0], cur_data[1], 0],
                                                ornObj=p.getQuaternionFromEuler([0, 0, 0]))
        # time.sleep(0.1)
        p.stepSimulation()
        time.sleep(0.1)

        

def extend_path(path,resolution=2):
    # TODO: extend path when length is different
    num_path = len(path)
    length_each_path = len(path[0])
    new_paths = []
    for i in range(num_path):
        current_path = path[i]
        tmp_path = []
        for j in range(length_each_path - 1):
            pt1 = np.array(current_path[j])
            pt2 = np.array(current_path[j+1])

            
            # Insert points using linspace
            new_points = np.linspace(pt1, pt2, resolution + 2)[1:-1]
            
            # Convert the new points to a list of lists
            new_points = new_points.tolist()
            
            # Extend the new_path with the current points and the new points
            tmp_path.extend([pt1.tolist()] + new_points)
        tmp_path.append(current_path[-1])
        new_paths.append(tmp_path)
    return new_paths

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map")
    parser.add_argument("schedule", help="schedule for agents")
    args = parser.parse_args()
    map_yaml = args.map
    schedule_yaml = args.schedule

    robots, schedule = create_env(map_yaml, schedule_yaml)

    # print(" origin path")
    # for i,j in enumerate(schedule):
    #     print(f"{i} is {j}")
    # print()
    # print(" new path")
    # for i,j in enumerate(extend_path(schedule)):
    #     print(f"{i} is {j}")
    # print()
    move_robot(robots, schedule)
    input("the simulation is end")

if __name__ == "__main__":
    main()