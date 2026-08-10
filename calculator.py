import numpy as np

import matplotlib.pyplot as plt

# width of the hub in meters
w = 1.05918

# height of the hub in meters
h = 1.8288

# gravity
g = 9.81

# m/s
launch_speed = 2.0
max_launch_speed = 11.0

# degrees
launch_angle = 47.5
max_launch_angle = 85.0

# m
distance = 3.0

# m (not sure what this is yet, have to change)
shooter_height = 0.4

# calculates whether the trajectory will allow for the ball to make it into the hub
# v_0 is the launch velocity, theta is the launch angle in radians, d is the distance from the front of the hub,
# and launch height is the height the fuel is launched at (this is constant but im not sure what this is currently)
def withinRange(v_0, theta, d, launch_height):
    discriminant = (v_0**2)*(np.sin(theta)**2)-4*(-0.5*g)*(launch_height-h)
    
    if discriminant < 0:
        return False, None
    
    t_0 = (v_0*np.sin(theta)+np.sqrt(discriminant))/(g)

    x_position = x_pos(v_0, theta, t_0)

    return (x_position >= d and x_position <= d+w), t_0

# calculates the x-position using kinematics
# v_0 is launch velocity, theta is launch angle in radians, t is time
def x_pos(v_0, theta, t):
    return v_0*np.cos(theta)*t

# calculates the y-position using kinematics
# v_0 is launch velocity, theta is launch angle in radians, launch_height is the launch height (lol), t is time
def y_pos(v_0, theta, launch_height, t):
    return launch_height+(v_0*np.sin(theta)*t)-((0.5*g)*(t**2))

plt.figure(figsize=(6, 6))

plt.vlines(x=distance, ymin=0, ymax=h)
plt.vlines(x=distance+w, ymin=0, ymax=h)

while launch_angle <= max_launch_angle:
    while launch_speed <= max_launch_speed:

        inRange = withinRange(launch_speed, np.radians(launch_angle), distance, shooter_height)

        if inRange[0]:
            t = np.linspace(0, inRange[1], 100)

            x = launch_speed*np.cos(np.radians(launch_angle))*t
            y = shooter_height+(launch_speed*np.sin(np.radians(launch_angle))*t)-(0.5*g)*(t**2)

            plt.plot(x, y)

        launch_speed += 0.5
    
    launch_speed = 2.0

    launch_angle += 1


plt.show()