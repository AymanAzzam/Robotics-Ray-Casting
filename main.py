import cv2 as cv
from numpy.core.fromnumeric import shape

from utility import *
from constants import *

def ray_helper(map_gray, start_x, start_y, theta, x, y, step):
    x_new = x
    y_new = y
    x, y = motion(start_x, start_y, theta, step * MOTION_STEP)
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
    for i in range(0, opening_angle, angle_step):
        end_pose = motion(start_x, start_y, angle, MAX_MEASURE)
        end_x = end_pose[0]
        end_y = end_pose[1]
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

def assignment_4_2(map_gray, z, opening_angle=250, angle_step=2):    
    likelihood_field = cv.GaussianBlur(255 - map_gray,(5,5), 1)
    array_2d = [[0 for i in range(len(likelihood_field[0]))] for j in range(len(likelihood_field))]
    cv.imshow("likelihood Field", likelihood_field)
    likelihood_field = likelihood_field.tolist()
    cv.waitKey(0)
    x_size, y_size = len(likelihood_field[0]), len(likelihood_field) 
    for y in range(0, y_size, 10):
        for x in range(0, x_size, 5):
            for theta in range(0, 360, 30):
                prop = 1
                angle = theta - opening_angle / 2
                for step in range(0, int(opening_angle / angle_step), 1):
                    end_x, end_y = motion(x, y, angle, z[step] / PIXEL_SIZE)
                    if end_y >= len(likelihood_field) or end_x >= len(likelihood_field[0]) or \
                                                                end_y < 0 or end_x < 0:
                        temp = 0
                    else:
                        prop *= likelihood_field[end_y][end_x]
                    angle += angle_step
                array_2d[y][x] = max(array_2d[y][x], prop)
    maximum = max(map(max, array_2d))
    position = -1, -1
    if maximum > 0: 
        array_2d = [[(x / maximum) * 255 for x in y] for y in array_2d]
        array_2d = np.array(array_2d)
        cv.imshow("end point model", array_2d)
        cv.waitKey(0)
        for y in range(len(array_2d)):
            for x in range(len(array_2d[0])):
                if array_2d[y][x] == 255:
                    position = x, y 
    print()
    print("Robot position (x, y) = ", position)
    return position

def assignment_4_1(map_gray, robot_pose, opening_angle=250, angle_step=2):
    z = measurements(map_gray, robot_pose, opening_angle, angle_step)
    z_formatted = [round(num, 3) for num in z]
    print()
    print("measurements(cm) = ", z_formatted)
    map_gray = draw_robot(map_gray, robot_pose)
    cv.imshow("map", map_gray)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return z
    
if __name__ == "__main__":
    map_image = read_image('Assignment_04_Grid_Map.png')
    # robot_pose = find_empty_space(map)
    robot_pose = [480, 110, 180]
    if check_robot_position(map_image, robot_pose):
        z = assignment_4_1(np.array(map_image), robot_pose, opening_angle=250, angle_step=2)
        position = assignment_4_2(map_image, z, opening_angle=250, angle_step=2)
        print()
        cv.destroyAllWindows()
    else:
        print("The robot inside the wall !")