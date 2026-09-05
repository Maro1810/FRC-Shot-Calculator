import numpy as np

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

# width of the funnel of the hub in meters
w = 1.05918

# height of the hub in meters
h = 1.8288

# gravity
g = 9.81

# m/s
# launch_speed = 1.0
# max_launch_speed = 12

# degrees
# launch_angle = 47.5
# max_launch_angle = 85.0

# m 
distance = 4

# m
front_clearance = 0.45

# m
back_clearance = 0.15

# m
top_clearance = 0.16

# m (not sure what this is yet, have to change)
shooter_height = 0.4

# +/- x m/s
velocity_uncertainty = 0.1

# +/- x radians
angle_uncertainty = 0.017

vel_error_weight = 0.5
dx_weight = 0.3
tof_weight = 0.2

plt.style.use('dark_background')

def sec(theta):
    return 1/(np.cos(theta))

# calculates whether the trajectory will allow for the ball to make it into the hub
# v_0 is the launch velocity, theta is the launch angle in radians, d is the distance from the front of the hub,
# and launch height is the height the fuel is launched at (this is constant but im not sure what this is currently)
def withinRange(v_0, theta, d, launch_height):
    discriminant = (v_0**2)*(np.sin(theta)**2)-4*(-0.5*g)*(launch_height-h)
    
    if discriminant < 0:
        return False, None, 0
    
    t_0 = (v_0*np.sin(theta)+np.sqrt(discriminant))/(g)
    t_1 = (d)/(v_0*np.cos(theta))

    y_height = y_pos(v_0, theta, launch_height, t_1)

    x_position = x_pos(v_0, theta, t_0)

    return (x_position >= (d+front_clearance) and x_position <= (d+w)-back_clearance and y_height >= (h+top_clearance)), t_0, x_position

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

def dx(x, v, theta):
    return (np.abs(dxdv(x, v, theta))*velocity_uncertainty)+(np.abs(dxd0(x, v, theta))*angle_uncertainty)

fig, axes = plt.subplots(1, 2, figsize=(10, 6))

axdistance = fig.add_axes((0.25, 0.05, 0.65, 0.03))

distance_slider = Slider(
    ax=axdistance,
    label='Distance',
    valmin=1,
    valmax=6,
    valinit=1,
)

def calculate_shot(current_distance, prev_angle=None):
    launch_speed = 1.0
    max_launch_speed = 12

    launch_angle = 55
    max_launch_angle = 84

    # axes[0].vlines(x=current_distance, ymin=0, ymax=h)
    # axes[0].vlines(x=current_distance+w, ymin=0, ymax=h)
    # axes[0].hlines(y=1.7, xmin=current_distance, xmax=current_distance+w)

    angles = []
    min_speeds = []
    max_speeds = []

    min_tof = 10000000
    max_tof = -1000000

    min_clearance = float('inf')
    max_clearance = float('-inf')

    while launch_angle <= max_launch_angle:

        speeds = []
        times = []

        while launch_speed <= max_launch_speed:

            inRange = withinRange(launch_speed, np.radians(launch_angle), current_distance, shooter_height)

            if inRange[0]:
                speeds.append(launch_speed)

                t = np.linspace(0, inRange[1], 100)

                times.append(t)

                curr_dx = dx(inRange[2], launch_speed, np.radians(launch_angle))
            
                m1 = inRange[2] - current_distance
                m2 = (current_distance+w) - inRange[2]
            
                clearance = min(m1, m2) - curr_dx

                if (inRange[1] < min_tof):
                    min_tof = inRange[1]

                if (inRange[1] > max_tof):
                    max_tof = inRange[1]

                if (clearance < min_clearance):
                    min_clearance = clearance
                
                if (clearance > max_clearance):
                    max_clearance = clearance

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

            # axes[0].plot(x1, y1, color='red', linewidth=0.3)
            # axes[0].plot(x2, y2, color='green', linewidth=0.3)
# 
            angles.append(launch_angle)
            min_speeds.append(min_velocity)
            max_speeds.append(max_velocity)
    
        launch_speed = 1.0

        launch_angle += 0.5

    upper_coeff = np.polyfit(angles, max_speeds, 4)

    lower_coeff = np.polyfit(angles, min_speeds, 4)

    difference = upper_coeff-lower_coeff

    # def upper_bound(x):
    #     return upper_coeff[0]*(x**4)+upper_coeff[1]*(x**3)+upper_coeff[2]*(x**2)+upper_coeff[3]*(x)+upper_coeff[4]

    # def lower_bound(x):
    #     return lower_coeff[0]*(x**4)+lower_coeff[1]*(x**3)+lower_coeff[2]*(x**2)+lower_coeff[3]*(x)+lower_coeff[4]

    def difference_func(x):
        return difference[0]*(x**4)+difference[1]*(x**3)+difference[2]*(x**2)+difference[3]*(x)+difference[4]

    max_margin = -10000
    min_margin = 10000

    launch_speed = 1.0
    launch_angle = min(angles)

    counter = min(angles)

    while (counter <= 85):

        if (difference_func(counter) > max_margin):
            max_margin = difference_func(counter)

        if (difference_func(counter) < min_margin):
            min_margin = difference_func(counter)

        counter += 0.1

    max_score = -100000
    score_speed = 0
    score_angle = 0

    while launch_angle <= max_launch_angle:
        while launch_speed <= max_launch_speed:
            inRange = withinRange(launch_speed, np.radians(launch_angle), current_distance, shooter_height)
        
            if inRange[0]:
                curr_dx = dx(inRange[2], launch_speed, np.radians(launch_angle))
            
                m1 = inRange[2] - current_distance
                m2 = (current_distance+w) - inRange[2]
            
                clearance = min(m1, m2) - curr_dx

                velocity_margin_score = (difference_func(launch_angle)-min_margin)/(max_margin-min_margin)
                clearance_score = (clearance-min_clearance)/(max_clearance-min_clearance)
                tof_score = (max_tof-inRange[1])/(max_tof-min_tof)

                score = (vel_error_weight*velocity_margin_score)+(dx_weight*clearance_score)+(tof_weight*tof_score)

                if prev_angle is not None:
                    jump_penalty = 0.01 * abs(launch_angle - prev_angle)
                    score -= jump_penalty

                if score > max_score:
                    max_score = score
                    score_speed = launch_speed
                    score_angle = launch_angle

            launch_speed += 0.01

        launch_speed = 1.0

        launch_angle += 0.5

    # axes[1].plot(angles, max_speeds, 'o', linestyle="-", color='green')
    # axes[1].plot(angles, min_speeds, 'o', linestyle="-", color='red')

    val = withinRange(score_speed, np.radians(score_angle), current_distance, shooter_height)[1]

    t = np.linspace(0, val, 100)

    x_optimal3 = score_speed*np.cos(np.radians(score_angle))*t
    y_optimal3 = shooter_height+(score_speed)*np.sin(np.radians(score_angle))*t+(-0.5*g)*(t**2)

    # axes[0].plot(x_optimal3, y_optimal3, linewidth=3, color='yellow')

    # axes[1].fill_between(angles, min_speeds, max_speeds, color='purple', alpha=0.8)

    return f"distance: {current_distance} m \nangle: {score_angle} deg \nspeed: {round(score_speed, 3)} m/s\n", score_angle, score_speed

j = 1

distances = []
speeds = []
angles = []

def update(val):
    return 0

while (j <= 6):

    distances.append(round(j, 3))

    if j == 1:
        with open("shots.txt", "w", encoding="utf-8") as file:
            file.write(calculate_shot(round(j, 3))[0])

            angles.append(calculate_shot(round(j, 3))[1])
            speeds.append(calculate_shot(round(j, 3))[2])

            previous_angle = calculate_shot(round(j, 3))[1]

    else:
        with open("shots.txt", "a", encoding="utf-8") as file:
            file.write("\n" + calculate_shot(round(j, 3), previous_angle)[0])

            angles.append(calculate_shot(round(j, 3), previous_angle)[1])
            speeds.append(calculate_shot(round(j, 3), previous_angle)[2])
            
            previous_angle = calculate_shot(round(j, 3), previous_angle)[1]

    j += 0.1

print(distances)
print(angles)
print(distances)

plt.show()