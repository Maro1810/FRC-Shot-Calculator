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
max_launch_speed = 12

# degrees
launch_angle = 47.5
max_launch_angle = 85.0

# m 
distance = 3

# m
front_clearance = 0.2

# m
back_clearance = 0.05

# m
top_clearance = 0.16

# m (not sure what this is yet, have to change)
shooter_height = 0.4

plt.style.use('dark_background')

def sec(theta):
    return 1/(np.cos(theta))

# calculates whether the trajectory will allow for the ball to make it into the hub
# v_0 is the launch velocity, theta is the launch angle in radians, d is the distance from the front of the hub,
# and launch height is the height the fuel is launched at (this is constant but im not sure what this is currently)
def withinRange(v_0, theta, d, launch_height):
    discriminant = (v_0**2)*(np.sin(theta)**2)-4*(-0.5*g)*(launch_height-h)
    
    if discriminant < 0:
        return False, None
    
    t_0 = (v_0*np.sin(theta)+np.sqrt(discriminant))/(g)
    t_1 = (distance)/(v_0*np.cos(theta))

    y_height = y_pos(v_0, theta, launch_height, t_1)

    x_position = x_pos(v_0, theta, t_0)

    return (x_position >= (d+front_clearance) and x_position <= (d+w)-back_clearance and y_height >= (h+top_clearance)), t_0

# calculates the x-position using kinematics
# v_0 is launch velocity, theta is launch angle in radians, t is time
def x_pos(v_0, theta, t):
    return v_0*np.cos(theta)*t

# calculates the y-position using kinematics
# v_0 is launch velocity, theta is launch angle in radians, launch_height is the launch height (lol), t is time
def y_pos(v_0, theta, launch_height, t):
    return launch_height+(v_0*np.sin(theta)*t)-((0.5*g)*(t**2))

def dxdv(x, v, theta):
    numerator = -g*(x**2)*sec(theta)
    denominator = (v**3)*(np.tan(theta))-(x*g*v*sec(theta))

    return numerator/denominator

def dxd0(x, v, theta):
    numerator = (g*(x**2))*(sec(theta)*np.tan(theta))-2*(v**2)*(x)*(sec(theta)**2)
    denominator = (2*(v**2)*np.tan(theta))-(2*x*g*sec(theta))

    return numerator/denominator

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
    
    if len(speeds) != 0:     
        min_velocity = min(speeds)
        max_velocity = max(speeds)

        min_time = speeds.index(min_velocity)
        max_time = speeds.index(max_velocity)

        x1 = min_velocity*np.cos(np.radians(launch_angle))*times[min_time]
        y1 = shooter_height+(min_velocity*np.sin(np.radians(launch_angle))*times[min_time])-(0.5*g)*(times[min_time]**2)

        x2 = max_velocity*np.cos(np.radians(launch_angle))*times[max_time]
        y2 = shooter_height+(max_velocity*np.sin(np.radians(launch_angle))*times[max_time])-(0.5*g)*(times[max_time]**2)

        axes[0].plot(x1, y1, color='red', linewidth=0.3)
        axes[0].plot(x2, y2, color='green', linewidth=0.3)

        angles.append(launch_angle)
        min_speeds.append(min_velocity)
        max_speeds.append(max_velocity)
    
    launch_speed = 1.0

    launch_angle += 0.5

upper_coeff = np.polyfit(angles, max_speeds, 4)

# x_vals = np.linspace(47.5, 85, 100)
# y_vals = np.polyval(upper_coeff, x_vals)

# axes[1].plot(x_vals, y_vals)

lower_coeff = np.polyfit(angles, min_speeds, 4)

# x_vals2 = np.linspace(47.5, 85, 100)
# y_vals2 = np.polyval(lower_coeff, x_vals2)

# axes[1].plot(x_vals2, y_vals2)

difference = upper_coeff-lower_coeff

def upper_bound(x):
    return upper_coeff[0]*(x**4)+upper_coeff[1]*(x**3)+upper_coeff[2]*(x**2)+upper_coeff[3]*(x)+upper_coeff[4]

def lower_bound(x):
    return lower_coeff[0]*(x**4)+lower_coeff[1]*(x**3)+lower_coeff[2]*(x**2)+lower_coeff[3]*(x)+lower_coeff[4]

def difference_func(x):
    return difference[0]*(x**4)+difference[1]*(x**3)+difference[2]*(x**2)+difference[3]*(x)+difference[4]

max = -10000
optimal_angle = 47.5

counter = 47.5

while (counter <= 85):
    if (difference_func(counter) > max):
        max = difference_func(counter)
        optimal_angle = counter

    counter += 0.1

curr = lower_bound(optimal_angle)
vertical_upper = upper_bound(optimal_angle)

horiz = ((vertical_upper-curr)/2)+curr

max_2 = -10000
optimal_speed = horiz

axes[1].vlines(x=optimal_angle, ymin=5, ymax=15)
axes[1].hlines(y=horiz, xmin=60, xmax=90)

axes[1].plot(angles, max_speeds, 'o', linestyle="-", color='green')
axes[1].plot(angles, min_speeds, 'o', linestyle="-", color='red')

val = withinRange(optimal_speed, np.radians(optimal_angle), distance, shooter_height)[1]

t = np.linspace(0, val, 100)

print(val)

x_optimal = optimal_speed*np.cos(np.radians(optimal_angle))*t
y_optimal = shooter_height+(optimal_speed)*np.sin(np.radians(optimal_angle))*t+(-0.5*g)*(t**2)

axes[0].plot(x_optimal, y_optimal, linewidth=3, color='blue')

plt.fill_between(angles, min_speeds, max_speeds, color='purple', alpha=0.8)

plt.show()