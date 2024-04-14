import numpy as np
import colorsys
import matplotlib.pyplot as plt
import yaml

import argparse

import pybullet as p
import pybullet_data


ROBOT_URDF_PATH = "/resources/robots/turtlebot/turtlebot.urdf"
PLANE_URDF_PATH = "/resources/object/plane/plane.urdf"


def create_obs_wall(x, y, demension):
    halfExtents = demension  # Half of the width, height, and depth of the box
    boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents, rgbaColor=[0.5, 0.5, 0.5, 1])
        # Set the initial position and orientation of the box
    boxPosition = [x, y, 0]  # Position (x, y, z)
    boxOrientation = p.getQuaternionFromEuler([0, 0, 0])  # Orientation in quaternion format (roll, pitch, yaw)
    p.createMultiBody(baseMass=1, baseCollisionShapeIndex=boxId, baseVisualShapeIndex=visualShapeId,
                                basePosition=boxPosition, baseOrientation=boxOrientation)

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
    for i in range(num_robots):
        data = map["agents"][i]
        r, g, b = colors[i]
        robot_id = p.loadURDF(ROBOT_URDF_PATH, basePosition=[data["start"][0], data["start"][1], 0.1])
        p.changeVisualShape(robot_id, 14, rgbaColor=[r, g, b, 1])  
        p.changeVisualShape(robot_id, 19, rgbaColor=[r, g, b, 1]) 
        p.changeVisualShape(robot_id, 26, rgbaColor=[r, g, b, 1])  
        robots.append(robot_id)

        visualShapeId = p.createVisualShape(shapeType=p.GEOM_CYLINDER, radius=0.1, rgbaColor=[r, g, b, 0.5])
        boxPosition = [data["goal"][0], data["goal"][1], 0]  # Position (x, y, z)
        boxOrientation = p.getQuaternionFromEuler([0, 0, 0])  # Orientation in quaternion format (roll, pitch, yaw)
        p.createMultiBody(baseMass=1, baseVisualShapeIndex=visualShapeId,
                                basePosition=boxPosition, baseOrientation=boxOrientation)
        
    return robots, schedule

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
    pass
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map")
    parser.add_argument("schedule", help="schedule for agents")
    args = parser.parse_args()
    map_yaml = args.map
    schedule_yaml = args.schedule

    robots, schedule = create_env(map_yaml, schedule_yaml)

    print(schedule["schedule"][0])
    input()

if __name__ == "__main__":
    main()