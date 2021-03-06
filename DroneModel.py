# models a 2D drone

import sys
import numpy as np
import matplotlib.pyplot as plt
import random
from math import cos, sin, pi, atan, atan2, sqrt, fabs

class Drone:
    def __init__(self, weight=0.1, motor_max_power=2, drone_length=3, x=0, y=0, theta=0, center_to_prop=1):
        self.drone_weight = weight              # kg
        self.motor_max_power = motor_max_power  # lift (N)
        self.drone_length = drone_length        # meters
        self.IMU_data = [-9.8, 0.0, 0.0]        # m/s^2 (not used yet)
        self.goal = [0.0, 0.0]
        self.center_to_prop = center_to_prop
        fig = plt.figure(1)
        fig.canvas.mpl_connect('button_press_event', self.click)        
        self.update_pose(x,y,theta)

    def update_pose(self, xpos, ypos, theta=0):
        self.x = xpos
        self.y = ypos
        self.theta = theta

    # transformation matrix
    def tm(self, theta):
        return np.array(
            [[cos(theta), -sin(theta), self.x],
             [sin(theta), cos(theta), self.y],
             [0,0,1]]
        )

    def plot(self, theta = 0, goal=[], xlims=[-10,10], ylims=[-10,10], wind=0):
        plt.ion()
        plt.cla()
        l = self.drone_length/2
        h = self.drone_length/10.0
        pw = self.drone_length/4.0
        drone_points = [[-l+pw, h, 1], [-l, h, 1], [-l, 0, 1], [l, 0, 1], [l, h, 1], [l-pw, h, 1]]

        tx, ty = [], []
        for point in drone_points:
            tp = np.matmul(self.tm(self.theta), np.array(point))
            tx.append(tp[0])
            ty.append(tp[1])

        # plot drone
        plt.plot(tx[0:2], ty[0:2], tx[2:4], ty[2:4], tx[4:6], ty[4:6])

        plt.plot(self.goal[0], self.goal[1],'b+')

        height = fabs(ylims[1]-ylims[0])
        mid = (xlims[1] + xlims[0])/2.0
        half = fabs(xlims[0] - xlims[1])/2.0
        plt.arrow(mid, ylims[1]-0.05*height, wind*half/10.0, 0, head_width=height/60.0)

        # set limits
        plt.xlim(xlims[0], xlims[1])
        plt.ylim(ylims[0], ylims[1])
        plt.show()
        plt.pause(.01)

    def click(self, event):
        self.goal = [event.xdata, event.ydata]


# --------- helper functions ---------------
# subtracts theta2 from theta1
def ang_diff(theta1, theta2):
    return(theta1-theta2+pi)%(2*pi)-pi

# converts angle from degrees to radians
def to_radians(angle):
    return angle*(pi/180)

# forces bounds onto num
def clamp(num, min, max):
    if num > max:
        return max
    elif num < min:
        return min
    else:
        return num

# given the desired force components, figure out the angle and total thrust needed
def get_T_and_theta_goal(Tx, Ty, robot_weight, Fmax):
    g = 9.8
    Tmax = 2*Fmax           # because there are 2 motors
    thetag = atan2(Tx, Ty)
    T = sqrt(Tx*Tx + Ty*Ty)
    T = clamp(T, 0, Tmax)
    return T, -thetag

# given desired x and y force components, drone orientation, angular velocity,
# length and motor location, get the optimal motor thrust
# Tx, Ty: x and y force components
# m: drone mass
# theta: drone orientation with respect to y axis
# w: angular velocity
# l: drone length
# r: distance from drone centerpoint to motor center
# Fmax: max individual motor thrust
def get_F1_and_F2(Tx, Ty, m, theta, w, l, r, Fmax):
    Kw = 3
    Ka = 1.1
    T, theta_goal = get_T_and_theta_goal(Tx, Ty, m, Fmax)
    w_goal = Kw*ang_diff(theta_goal, theta)
    a = Ka*(w_goal - w)
    term = (a*m*l*l)/(24*r)
    F1 = term + T/2.0
    F2 = T/2 - term
    F1 = clamp(F1, 0, F1)
    F2 = clamp(F2, 0, F2)
    '''
    print("----------------------------")
    print("theta_goal: ", theta_goal)
    print("theta: ", theta)
    print("omega_goal: ", w_goal)
    print("omega: ", w)
    '''
    return F1, F2

def max(a, b):
    if a > b:
        return a
    else:
        return b

# basically a custom autoscale. Scales the graph limits to frame the
# drone and the waypoint around the point halfway between them
def get_graph_limits(pta, ptb, dlen):
    xdiff = ptb[0] - pta[0]
    ydiff = ptb[1] - pta[1]
    abx = fabs(xdiff)
    aby = fabs(ydiff)
    centerpoint = [0, 0]
    minlim = 5*dlen

    if xdiff > 0:
        centerpoint[0] = pta[0] + 0.5*abx
    elif xdiff < 0:
        centerpoint[0] = ptb[0] + 0.5*abx
    else:
        centerpoint[0] = pta[0]

    if ydiff > 0:
        centerpoint[1] = pta[1] + 0.5*aby
    elif ydiff < 0:
        centerpoint[1] = ptb[1] + 0.5*aby
    else:
        centerpoint[1] = pta[1]

    # wind -> window
    xwind = 0.75*abx
    ywind = 0.75*aby

    # make sure the window doesn't zoom in too much
    if xwind < minlim:
        xwind = minlim

    if ywind < minlim:
        ywind = minlim

    # make sure the window is large enough to include the drone
    if xwind < 0.5*abx + dlen:
        xwind = (0.75*abx + dlen)

    if ywind < 0.5*aby + dlen:
        ywind = (0.75*aby + dlen) 


    wind = max(xwind, ywind)

    xlims = [centerpoint[0] - wind, centerpoint[0] + wind]
    ylims = [centerpoint[1] - wind, centerpoint[1] + wind]

    return xlims, ylims 

# compute an approximation of the drag force given drone and wind velocity and drag coefficent
def drag_force(b, v, v_wind=0):
    v -= v_wind     # force decreases if wind and drone velocity are in the same direction
    # print("v = ", v, ", v_wind = ", v_wind)
    if fabs(v) < 20:
        return -b*v
    else:
        return -b*v*fabs(v)

# generates a random float in the range wind +/- amount
def change_wind(wind, amount):
    ran = random.random()
    num = ran*amount*2.0 - amount
    return wind + num

# computes the fastest the drone can go and still slow down before reaching the goal
def get_goal_max(vx, vy, dx, dy, Tmax, m, g):
    theta = atan2(vy, vx)
    ax = Tmax*sin(theta)/m

    # fg decreases acceleration if the drone is above the goal
    g = fabs(ag)
    Fg = 0
    if (dy > 0):
        Fg = -m * g
    else:
        Fg = m*g

    ay = (Tmax*cos(theta) + Fg)/m
    vx_max = sqrt(2.0*fabs(ax)*fabs(dx))
    vy_max = sqrt(2.0*fabs(ay)*fabs(dy))

    return vx_max, vy_max


if __name__ == "__main__":
    auto_mode = True   # switches between automatically updating the goal and relying on user input
    wind_on = False
    v_wind = 0.0
    if wind_on:
        v_wind = 1.0 # m/s


    # control constants, good values Kty = 0.3, Ktx = 0.1, Kiy = 0.01, Kw = 3, Ka = to_radians(45)/10
    Kty = .3
    Ktx = .3
    Kiy = 0.005
    Kix = 0.001
    Kw = 3
    Kv = 0.75
    theta_goal = 0
    vx_goal = 0
    vy_goal = 0
    yEsum = 0
    xEsum = 0

    # environment constants
    ag = -9.8
    b = 0.000      # drag coefficient (set to 0 to turn off air resistance)
    dt = 0.05

    # kinematic variables
    theta = 0.0     # difference between drone normal vector and vertical
    theta_max = to_radians(50) 
    omega = 0.0
    xx = 0.0
    xy = 0.0
    vx = 0.0
    vy = 0.0
    v = 0.0
    T = 0.0
    F1 = 0.0
    F2 = 0.0
    Tx = 0.0
    Ty = 0.0
    ax = 0.0
    ay = 0.0

    # miscellaneous
    iters = 0
    giter = 1

    # list of waypoints
    goals = [[10000, 10000], [100, -100], [77, 20], [0, 19], [-100, -100]]
    xl, yl = [-50, 50], [-50, 50]

    robert = Drone(drone_length=2, motor_max_power=3)
    max_lift = robert.motor_max_power * 2   # drone has two propellers

    # terminal velocity takes max thrust into account
    if b > 0:
        terminal_velocity_x = sqrt(fabs(max_lift/b))
        terminal_velocity_y_up = sqrt((fabs(max_lift) - robert.drone_weight*fabs(ag))/b)
        terminal_velocity_y_down = sqrt((fabs(max_lift) + robert.drone_weight*fabs(ag))/b)
    else: 
        terminal_velocity_x = 1000 # infinity
        terminal_velocity_y_up = 1000 # infinity
        terminal_velocity_y_down = 1000 # infinity

    if auto_mode:
        robert.goal = goals[0]

    # go
    while True:

        xEsum += (vx_goal - vx)
        yEsum += (vy_goal - vy)

        # integral term for if there is horizontal and vertical wind to account for non-zero steady-state thrust
        # gravity force is baked into y thrust automatically
        if wind_on:
            xIterm = Kix*xEsum
            yIterm = Kiy*yEsum
        else:
            xIterm = 0
            yIterm = 0

        # get the force vectors needed to get to the goal velocity
        Tx = Ktx*(vx_goal - vx) + xIterm
        Ty = Kty*(vy_goal - vy) + yIterm - robert.drone_weight*ag

        # get the ideal force and angle to get the desired x and y components of force (right now T updates instantaneously, theta doesn't)
        F1, F2 = get_F1_and_F2(Tx, Ty, robert.drone_weight, theta, omega, robert.drone_length, robert.center_to_prop, robert.motor_max_power)
        F1 = clamp(F1, 0, robert.motor_max_power)
        F2 = clamp(F2, 0, robert.motor_max_power)

        # the further from the goal, the faster we want to go
        vx_goal = (robert.goal[0] - xx) * Kv
        vy_goal = (robert.goal[1] - xy) * Kv

        # don't try to go faster than terminal velocity
        vx_goal = clamp(vx_goal, -terminal_velocity_x, terminal_velocity_x)
        vy_goal = clamp(vy_goal, -terminal_velocity_y_down, terminal_velocity_y_up)

        # also want to make sure it can never go faster than it could slow down in the distance before the goal
        # (actually don't want to exceed 70% of that speed)
        speed_factor = 0.7
        vx_goal_max, vy_goal_max = get_goal_max(vx, vy, xx-robert.goal[0], xy-robert.goal[1], robert.motor_max_power*2, robert.drone_weight, ag)
        vx_goal = clamp(vx_goal, -vx_goal_max*speed_factor, vx_goal_max*speed_factor)
        vy_goal = clamp(vy_goal, -vy_goal_max*speed_factor, vy_goal_max*speed_factor)

        # stop as quickly as possible if we are going faster than we can slow down
        if fabs(vx_goal_max) < fabs(vx):
            vx_goal = 0
            '''
            print("---------------------------------------")
            print("x overshoot")
            print("dx: ", xx-robert.goal[0])
            print("vxmax: ", vx_goal_max)
            print("vx_goal: ", vx_goal)
            print("vx: ", vx)
            '''
        if fabs(vy_goal_max) < fabs(vy):
            vy_goal = 0
            '''
            print("---------------------------------------")
            print("y overshoot")
            print("dy: ", xy-robert.goal[1])
            print("vy_goal_max: ", vy_goal_max)
            print("vy_goal: ", vy_goal)
            print("vy: ", vy)
            '''

        # get thrust and angular acceleration from motor forces
        alpha = (12*robert.center_to_prop*(-F2 + F1))/(robert.drone_weight*robert.drone_length**2)
        T = F1 + F2

        # update kinematics
        drag_force_x = drag_force(b, vx, v_wind)    # always in the opposite direction as v
        dx = drag_force(b, vx)
        drag_force_y = drag_force(b, vy)
        ax = (-T*sin(theta) + drag_force_x)/robert.drone_weight
        at = (T*cos(theta) + drag_force_y)/robert.drone_weight
        ay = at + ag
        vx += ax*dt
        vy += ay*dt
        xx += vx*dt
        xy += vy*dt
        omega += alpha
        theta += omega*dt

        # calculate speed for logging
        v = sqrt(vx**2 + vy**2)
        vmph = v * 2.24
        #print("speed = ", v, " mps, ", vmph, "mph")
        #print("wind speed = ", v_wind)

        # update drone
        robert.update_pose(xx, xy, theta)
        if auto_mode:
            xl, yl = get_graph_limits([xx, xy], robert.goal, robert.drone_length)
        robert.plot(xlims=xl, ylims=yl, wind=v_wind)
        iters += 1

        # if wind is on change wind slightly every three seconds
        if wind_on and iters*dt % 5 == 0:
            v_wind = change_wind(v_wind, 1)
            v_wind = clamp(v_wind, -10, 10)
        
        # in autonomous mode, go to next goal if the current one has been reached 
        if fabs(robert.goal[0] - xx) < 0.1 and fabs(robert.goal[1] - xy) < 0.1 and sqrt(vx**2 + vy**2) < 0.1 and auto_mode:
            robert.goal[0] = goals[giter][0]
            robert.goal[1] = goals[giter][1]

            # cycle back to the beginning of the goals list if the end has been reached
            if (giter >= len(goals) - 1):
                giter = 0
            else:
                giter += 1
        

