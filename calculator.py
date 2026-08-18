import numpy as np

import matplotlib.pyplot as plt

# width of the funnel of the hub in meters
w = 1.05918

# height of the hub in meters
h = 1.8288

# gravity
g = 9.81

# m/s
launch_speed = 1.0
max_launch_speed = 13.0

# degrees
launch_angle = 47.5
max_launch_angle = 85.0

# m
distance = 3.0

# m
margin = 0.2

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

    return (x_position >= (d+margin) and x_position <= d+w), t_0

# calculates the x-position using kinematics
# v_0 is launch velocity, theta is launch angle in radians, t is time
def x_pos(v_0, theta, t):
    return v_0*np.cos(theta)*t

# calculates the y-position using kinematics
# v_0 is launch velocity, theta is launch angle in radians, launch_height is the launch height (lol), t is time
def y_pos(v_0, theta, launch_height, t):
    return launch_height+(v_0*np.sin(theta)*t)-((0.5*g)*(t**2))

fig, axes = plt.subplots(1, 2, figsize=(10, 6))

axes[0].vlines(x=distance, ymin=0, ymax=h)
axes[0].vlines(x=distance+w, ymin=0, ymax=h)
axes[0].hlines(y=1.7, xmin=distance, xmax=distance+w)

angles = []
min_speeds = []
max_speeds = []

while launch_angle <= max_launch_angle:

    speeds = []
    times = []

    while launch_speed <= max_launch_speed:

        inRange = withinRange(launch_speed, np.radians(launch_angle), distance, shooter_height)

        if inRange[0]:
            speeds.append(launch_speed)

            t = np.linspace(0, inRange[1], 100)

            times.append(t)

        launch_speed += 0.01

    speeds = list(dict.fromkeys(speeds))
    
    if len(speeds) != 0:     
        min_velocity = min(speeds)
        max_velocity = max(speeds)

        min_time = speeds.index(min_velocity)
        max_time = speeds.index(max_velocity)

        x1 = min_velocity*np.cos(np.radians(launch_angle))*times[min_time]
        y1 = shooter_height+(min_velocity*np.sin(np.radians(launch_angle))*times[min_time])-(0.5*g)*(times[min_time]**2)

        x2 = max_velocity*np.cos(np.radians(launch_angle))*times[max_time]
        y2 = shooter_height+(max_velocity*np.sin(np.radians(launch_angle))*times[max_time])-(0.5*g)*(times[max_time]**2)

        axes[0].plot(x1, y1, color='red')
        axes[0].plot(x2, y2, color='green')

        angles.append(launch_angle)
        min_speeds.append(min_velocity)
        max_speeds.append(max_velocity)
    
    launch_speed = 2.0

    launch_angle += 0.5

# coefficients = np.polyfit(angles, max_speeds, 2)

# x_vals = np.linspace(47.5, 85, 100)
# y_vals = np.polyval(coefficients, x_vals)

# axes[1].plot(x_vals, y_vals)

axes[1].plot(angles, max_speeds, 'o', linestyle="-", color='green')
axes[1].plot(angles, min_speeds, 'o', linestyle="-", color='red')

plt.fill_between(angles, min_speeds, max_speeds, color='purple', alpha=0.3)

plt.show()