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
TANGENTIAL_SPEED = max_angular_speed*radio_wheel

# Functions
def turn_robot(angle,right_motor,left_motor):
    """
    **vel**: velocity of robot 
    Positive for rigth an negative to left
    """
    if angle > 0:
        left_speed = -max_angular_speed
        right_speed = max_angular_speed
    else:
        left_speed = max_angular_speed
        right_speed = -max_angular_speed

    angle_robot = angle
    radio_robot = 0.0400013
    # applying formula
    time = abs(angle_robot)
    print(angle_wheel)
    # control velocities 
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    delay_function(time)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def move_forward(distance):
    time = distance / TANGENTIAL_SPEED

    left_motor.setVelocity(-max_angular_speed)
    right_motor.setVelocity(-max_angular_speed)
    delay_function(time)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def delay_function(sec):
    current_time_1 = float(robot.getTime())

    delay_time = current_time_1 + 2
    while True:
        current_time_2 = float(robot.getTime())
        robot.step(1);
        if (current_time_2 >= delay_time):
            break
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
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)

    # AStar Algorithm
    GRID_WIDTH = 116
    GRID_HEIGHT = 39

    initial_node = (6,20)
    final_node = (115, 20)

    a_star = AStar()
    grid = np.zeros([GRID_WIDTH, GRID_HEIGHT])

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    move_forward(0.02)
    while robot.step(timestep) != -1:

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
            grid[initial_node[0] + 2, initial_node[1]] = -1

        if ds_right_value > 9.25:
            print("Wall Right: ", ds_right_value)
            grid[initial_node[0], initial_node[1] + 2] = -1

        if ds_left_value > 9.25:
            print("Wall Left: ", ds_left_value)
            grid[initial_node[0], initial_node[1] - 2] = -1

        # Analyzing data from light sensors
        if ls_center_value == 10:
            print("Light in front: ", ls_center_value)
            grid[initial_node[0] + 2, initial_node[1]] = -1

        if ls_right_value == 10:
            print("Light right: ", ls_right_value)
            grid[initial_node[0], initial_node[1] + 2] = -1

        if ls_left_value == 10:
            print("Light Left: ", ls_left_value)
            grid[initial_node[0], initial_node[1] - 2] = -1
