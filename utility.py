import cv2 as cv
import numpy as np

from constants import ROBOT_SIZE


# Read RGB image and convert it to gray-scale
def read_image(path):
    img = cv.imread(path, cv.IMREAD_GRAYSCALE)
    return img


# Move the robot in the direct direction
# x, y and move are the unit (in this code pixels)
def motion(x, y, theta, move):
    x_new = x + move * np.cos(theta * np.pi / 180)
    y_new = y + move * np.sin(theta * np.pi / 180)
    return int(x_new), int(y_new)


# Calculate the distance between two points
def calc_measurement(x0, y0, x1, y1):
    return np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)


# to handle out of bounds (it needs improvement)
def handle_map_range(map_gray, x, y):
    x = min(x, len(map_gray[0]) - 1)
    x = max(x, 0)
    y = min(y, len(map_gray) - 1)
    y = max(y, 0)
    return x, y


# Output:
#   True: when the robot in an empty space
#   False: when the robot inside the wall !
def check_robot_position(map_gray, pose):
    x = pose[0]
    y = pose[1]
    limit = int(ROBOT_SIZE / 2)
    for i in range(x - limit, x + limit + 1):
        for j in range(y - limit, y + limit + 1):
            if j >= len(map_gray) or i >= len(map_gray[0]) or map_gray[j][i] < 100:
                return False
    return True


# find the first empty position on the map to locate the robot
def find_empty_space(map_gray):
    print("Finding an empty space for the robot...")
    for x in range(0, len(map_gray[0])):
        for y in range(0, len(map_gray)):
            robot_pose = [x, y, 0]
            if check_robot_position(map_gray, robot_pose):
                print("Robot Pose: ", robot_pose)
                return robot_pose
