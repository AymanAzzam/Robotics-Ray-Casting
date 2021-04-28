import cv2 as cv

from utility import *
from constants import *

def ray_helper(map_gray, start_x, start_y, theta, x, y, step):
    x_new = x
    y_new = y
    x, y = motion(start_x, start_y, theta, step * MOTION_STEP)
    x, y = handle_map_range(map_gray, x, y)
    step +=1
    return x, y, x_new, y_new, step

# move the ray till it collide or rich the maximum position
def ray(map_gray, x, y, theta, end_x, end_y):
    start_x = x
    start_y = y
    step = 1
    if y <= end_y and x <= end_x:
        while (map_gray[y][x] > THRESHOLD and y < len(map_gray) and x < len(map_gray[0])
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            x, y, x_new, y_new, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    elif y >= end_y and x >= end_x:
        while (map_gray[y][x] > THRESHOLD and y >= 0 and x >= 0
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            x, y, x_new, y_new, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    elif y <= end_y and x >= end_x:
        while (map_gray[y][x] > THRESHOLD and y < len(map_gray) and x >= 0
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            x, y, x_new, y_new, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    elif y >= end_y and x <= end_x:
        while (map_gray[y][x] > THRESHOLD and y >= 0 and x < len(map_gray[0])
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            x, y, x_new, y_new, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    elif y <= end_y and x == end_x:
        x_new = x
        while (map_gray[y][x] < THRESHOLD and y < len(map_gray)
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            _, y, _, y_new, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    elif y >= end_y and x == end_x:
        x_new = x
        while (map_gray[y][x] < THRESHOLD and y >= 0
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            _, y, _, y_new, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    elif y == end_y and x >= end_x:
        y_new = y
        while (map_gray[y][x] < THRESHOLD and x >= 0
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            x, _, x_new, _, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    elif y == end_y and x <= end_x:
        y_new = y
        while (map_gray[y][x] < THRESHOLD and x > len(map_gray[0])
               and calc_measurement(x, y, start_x, start_y) <= MAX_MEASURE):
            x, _, x_new, _, step = ray_helper(map_gray, start_x, start_y, theta, x, y, step)
    return x_new, y_new


def measurements(map, pose, opening_angle=250, angle_step=2):
    start_x = pose[0]
    start_y = pose[1]
    start_theta = pose[2]

    angle = start_theta - opening_angle / 2
    z = []
    for i in range(0, opening_angle + 1, angle_step):
        end_pose = motion(start_x, start_y, angle, MAX_MEASURE)
        end_x = end_pose[0]
        end_y = end_pose[1]
        end_x, end_y = handle_map_range(map, end_x, end_y)
        x, y = ray(map, start_x, start_y, angle, end_x, end_y)
        # new_map = cv.line(map, (start_x, start_y), (end_x, end_y), (100), 2)
        new_map = cv.line(map, (start_x, start_y), (x, y), 200, 2)

        z.append(calc_measurement(start_x, start_y, x, y) * PIXEL_SIZE)
        angle += angle_step
    return z


# draw the robot and it's direction
def draw_robot(map_gray, pose):
    map_rgb = cv.cvtColor(map_gray, cv.COLOR_GRAY2RGB)
    x = pose[0]
    y = pose[1]
    theta = pose[2]
    limit = int(ROBOT_SIZE / 2)
    new_map = cv.rectangle(map_rgb, (x - limit, y - limit), (x + limit, y + limit), (0, 0, 255), -1)
    new_map = cv.line(new_map, (x, y), motion(x, y, theta, ROBOT_SIZE), (42, 52, 57), 2)
    return new_map


if __name__ == "__main__":
    map_image = read_image('Assignment_04_Grid_Map.png')
    # robot_pose = find_empty_space(map)
    robot_pose = [480, 110, 180]
    if check_robot_position(map_image, robot_pose):
        z = measurements(map_image, robot_pose)
        z_formatted = [round(num, 3) for num in z]
        print("measurements(cm) = ", z_formatted)
        map_image = draw_robot(map_image, robot_pose)
        cv.imshow("map", map_image)
        cv.waitKey(0)
        cv.destroyAllWindows()
    else:
        print("The robot inside the wall !")
