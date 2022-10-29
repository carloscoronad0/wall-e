"""wall-e-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

if __name__== "__main__":
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 24
    max_speed = 5.28
    
    left_motor = robot.getDevice('left_motor')
    right_motor = robot.getDevice('right_motor')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    right_sensor = robot.getDevice('distance_sensor_r')
    left_sensor = robot.getDevice('distance_sensor_l')
    front_sensor = robot.getDevice('distance_sensor')

    right_sensor.enable(200)
    left_sensor.enable(200)
    front_sensor.enable(200)
    
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        left_speed = -0.5 * max_speed
        right_speed = -0.5 * max_speed

        # control velocities 
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        print('------Distance sensor------')
        print('right:',right_sensor.getValue())
        print('left:',left_sensor.getValue())
        print('Front:',front_sensor.getValue())
