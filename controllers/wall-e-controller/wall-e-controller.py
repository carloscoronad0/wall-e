"""wall-e-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from a_star import AStar

robot = Robot()
max_angular_speed = 5 #rpm
left_motor = robot.getDevice('right_motor')
right_motor = robot.getDevice('left_motor')

radio_wheel = 0.03
TANGENTIAL_SPEED = max_angular_speed * radio_wheel

radio_robot = 0.0505013 #dstance beetween wheels / 2
# robot_rotational_speed = TANGENTIAL_SPEED/(2*radio_robot*math.pi)
# robot_rotational_speed = 0.5968116402163006
robot_rotational_speed = 0.44
# robot_angular_speed_in_degrees = robot_rotational_speed*360
# robot_angular_speed_in_degrees = 214.85219047786822
robot_angular_speed_in_degrees = robot_rotational_speed*360
# Functions
def move_forward(distance):
    time = distance / TANGENTIAL_SPEED

    left_motor.setVelocity(-max_angular_speed)
    right_motor.setVelocity(-max_angular_speed)
    delay_function(time)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def delay_function(sec):
    current_time_1 = float(robot.getTime())

    delay_time = current_time_1 + sec
    while True:
        current_time_2 = float(robot.getTime())
        robot.step(1);
        if (current_time_2 >= delay_time):
            break

def turn_robot(angle):
    """
    Positive for rigth an negative to left
    """
    if angle == 0:
        return

    if angle > 0:
        left_speed = -max_angular_speed
        right_speed = max_angular_speed
        robot_rotational_speed = 0.4529
    else:
        left_speed = max_angular_speed
        right_speed = -max_angular_speed
        robot_rotational_speed = 0.44

    angle_robot = abs(angle)
    # applying formula
    time = angle_robot/(robot_rotational_speed*360)
    print(time)
    # control velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    delay_function(time)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# Main
if __name__== "__main__":

    timestep = int(robot.getBasicTimeStep())
    # Engines
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # Sonars
    right_sensor = robot.getDevice('distance_sensor_r')
    left_sensor = robot.getDevice('distance_sensor_l')
    front_sensor = robot.getDevice('distance_sensor')

    right_sensor.enable(timestep)
    left_sensor.enable(timestep)
    front_sensor.enable(timestep)

    # LightSensor
    right_light_sensor = robot.getDevice('light_sensor_r')
    left_light_sensor = robot.getDevice('light_sensor_l')
    center_light_sensor = robot.getDevice('light_sensor_c')
    panel = robot.getDevice('panel')

    right_light_sensor.enable(timestep)
    left_light_sensor.enable(timestep)
    center_light_sensor.enable(timestep)
    panel.enable(timestep)

    # AStar Algorithm variables
    GRID_WIDTH = 116
    GRID_HEIGHT = 39

    initial_node = (6,20)
    final_node = (110, 20)

    a_star = AStar()
    grid = np.zeros([GRID_WIDTH, GRID_HEIGHT])

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    no_obstacles_found = True
    while robot.step(timestep) != -1:

        # Capture values
        ds_right_value = right_sensor.getValue()
        ds_left_value = left_sensor.getValue()
        ds_front_value = front_sensor.getValue()

        ls_panel_value = panel.getValue()
        ls_right_value = right_light_sensor.getValue()
        ls_left_value = left_light_sensor.getValue()
        ls_center_value = center_light_sensor.getValue()

        # Analyzing data from distance sensors
        if ds_front_value > 9.25:
            print("Wall In Front: ", ds_front_value)
            if initial_node[0] + 2 < GRID_WIDTH:
                if grid[initial_node[0] + 2, initial_node[1]] == 0:
                    grid[initial_node[0] + 2, initial_node[1]] = -1
                    no_obstacles_found = False

        if ds_right_value > 9.25:
            print("Wall Right: ", ds_right_value)
            if initial_node[0] + 2 < GRID_WIDTH and initial_node[1] + 1 < GRID_HEIGHT:
                if grid[initial_node[0] + 2, initial_node[1] + 1] == 0:
                    grid[initial_node[0] + 2, initial_node[1] + 1] = -1
                    no_obstacles_found = False

        if ds_left_value > 9.25:
            print("Wall Left: ", ds_left_value)
            if initial_node[0] + 2 < GRID_WIDTH and initial_node[1] - 1 >= 0:
                if grid[initial_node[0] + 2, initial_node[1] - 1] == 0:
                    grid[initial_node[0] + 2, initial_node[1] - 1] = -1
                    no_obstacles_found = False


        # Analyzing data from light sensors
        if ls_center_value == 10:
            print("Light in front: ", ls_center_value)
            if initial_node[0] + 3 < GRID_WIDTH and initial_node[1] - 1 >= 0 and initial_node[1] + 1 < GRID_HEIGHT:
                if grid[initial_node[0] + 2, initial_node[1]] == 0:
                    grid[initial_node[0] + 2, initial_node[1]] = -2
                    grid[initial_node[0] + 2, initial_node[1] - 1] = -1
                    grid[initial_node[0] + 2, initial_node[1] + 1] = -1
                    grid[initial_node[0] + 1, initial_node[1]] = -1
                    grid[initial_node[0] + 3, initial_node[1]] = -1
                    no_obstacles_found = False

        if ls_right_value == 10:
            print("Light right: ", ls_right_value)
            if initial_node[0] + 3 < GRID_WIDTH and initial_node[1] + 2 < GRID_HEIGHT:
                if grid[initial_node[0] + 2, initial_node[1] + 1] == 0:
                    grid[initial_node[0] + 2, initial_node[1] + 1] = -2
                    grid[initial_node[0] + 2, initial_node[1]] = -1
                    grid[initial_node[0] + 2, initial_node[1] + 2] = -1
                    grid[initial_node[0] + 1, initial_node[1] + 1] = -1
                    grid[initial_node[0] + 3, initial_node[1] + 1] = -1
                    no_obstacles_found = False

        if ls_left_value == 10:
            print("Light Left: ", ls_left_value)
            if initial_node[0] + 3 < GRID_WIDTH and initial_node[1] - 2 >= 0:
                if grid[initial_node[0] + 2, initial_node[1] - 1] == 0:
                    grid[initial_node[0] + 2, initial_node[1] - 1] = -2
                    grid[initial_node[0] + 2, initial_node[1] - 2] = -1
                    grid[initial_node[0] + 2, initial_node[1]] = -1
                    grid[initial_node[0] + 1, initial_node[1] - 1] = -1
                    grid[initial_node[0] + 3, initial_node[1] - 1] = -1
                    no_obstacles_found = False

        if no_obstacles_found:
            move_forward(0.02)
            initial_node = (initial_node[0] + 1, initial_node[1])
            print("Moving,no obstacles!!, Initial Node: ", initial_node)
        else:
            a_star.clean_dictionaries()
            path = a_star.search_for_optimal_path(initial_node, final_node, grid)
            next_node = path[-2]

            if next_node == final_node:
                print("Goal Node Reached!")
            else:
                # Compare to move
                if initial_node[0] - next_node[0] == 0:
                    if initial_node[1] - next_node[1] > 0:
                        turn_robot(90)
                        delay_function(0.5)
                        move_forward(0.02)
                        delay_function(0.5)
                        turn_robot(-90)
                        print("Move up (y - 1)")
                    else:
                        turn_robot(-90)
                        delay_function(0.5)
                        move_forward(0.02)
                        delay_function(0.5)
                        turn_robot(90)
                        print("Move down (y + 1)")
                else:
                    print("Move Forward")
                    move_forward(0.02)
                no_obstacles_found = True
