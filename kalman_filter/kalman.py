import kinem
import math
from matplotlib import pyplot as plt
import numpy as np

r = kinem.Filter(0.04, 0.1, 0.15)
r.setPos(0, 0, math.pi/3)
r.setDeltaT(0.01)

r_interfere = kinem.Filter(0.04, 0.1, 0.15)
r_interfere.setPos(0, 0, math.pi/3)
r_interfere.setDeltaT(0.01)

# body
phi = np.arange(0,2*np.pi,0.1)
x_c = r.R*np.cos(phi)
y_c = r.R*np.sin(phi)

# plan
plan = np.matrix([0, 0, 0], dtype=float)
plan = np.append(plan, [[0.5, 30, 28]], axis=0)
plan = np.append(plan, [[1.5, 50, 40]], axis=0)
plan = np.append(plan, [[0.5, 10, 10]], axis=0)
plan = np.append(plan, [[1, 40, 54]], axis=0)

plt.figure(figsize=(7, 5))

c_first = plt.subplot(2, 2, 1)
c_first.set_xlim(-2, 2)
c_first.set_ylim(-2, 2)

c_sec = plt.subplot(2, 2, 2, sharey=c_first, sharex=c_first)
c_third = plt.subplot(2, 2, 3, sharey=c_first, sharex=c_first)
c_four = plt.subplot(2, 2, 4, sharey=c_first, sharex=c_first)

#moving function
def move(t, wl, wr):
    r.setV(wl, wr)
    r_interfere.setV(wl, wr)
    while (t > 0):
        t -= r.dT

        r.setNoise(0, 0)
        r.nextPos()

        r_interfere.setNoise(0.1, 0.1)
        r_interfere.nextPos()
       
        x_r = r.px + x_c
        y_r = r.py + y_c

        x_r_interfere = r_interfere.px + x_c
        y_r_interfere = r_interfere.py + y_c

        # heading
        x2 = r.px + (r.R + 0.05)*math.cos(r.theta)
        y2 = r.py + (r.R + 0.05)*math.sin(r.theta)

        x2_noise = r_interfere.px + (r_interfere.R + 0.05)*math.cos(r_interfere.theta)
        y2_noise = r_interfere.py + (r_interfere.R + 0.05)*math.sin(r_interfere.theta)

        # li thuyet
        r_path, = c_first.plot(r.real_tracker[0:len(r.real_tracker), 2], r.real_tracker[0:len(r.real_tracker), 3], color="red")
        r_heading, = c_first.plot([r.px, x2], [r.py, y2],color="black")
        r_body, = c_first.plot(x_r, y_r, color="black")

        # thuc te
        r_interfere_path, = c_sec.plot(r_interfere.real_tracker[0:len(r_interfere.real_tracker), 2], r_interfere.real_tracker[0:len(r_interfere.real_tracker), 3], color="green")
        r_interfere_heading, = c_sec.plot([r_interfere.px, x2_noise], [r_interfere.py, y2_noise],color="black")
        r_interfere_body, = c_sec.plot(x_r_interfere, y_r_interfere, color="black")

        # do luong
        r_interfere_mesurement, = c_third.plot(r_interfere.measurement_tracker[0:len(r_interfere.measurement_tracker),0], r_interfere.measurement_tracker[0:len(r_interfere.measurement_tracker),1], color="yellow")

        # uoc tinh
        r_interfere_estimate, = c_four.plot(r_interfere.estimate[0:len(r_interfere.estimate),0], r_interfere.estimate[0:len(r_interfere.estimate),1], color="purple")
        #plt.gca().set_aspect('equal')
        plt.pause(0.000001)
        r_body.remove()
        r_heading.remove()

        r_interfere_body.remove()
        r_interfere_heading.remove()
        
        if t - 0 > 0.00000001:
            r_path.remove()
            r_interfere_path.remove()
            r_interfere_mesurement.remove()
            r_interfere_estimate.remove()
            
    

def perform(plan):
    for i in plan:
        move(i[0,0], i[0,1], i[0,2])

perform(plan)       

def lastPos():

    x_r = r.px + x_c
    y_r = r.py + y_c

    x_r_interfere = r_interfere.px + x_c
    y_r_interfere = r_interfere.py + y_c

    # heading
    x2 = r.px + (r.R + 0.05)*math.cos(r.theta)
    y2 = r.py + (r.R + 0.05)*math.sin(r.theta)

    x2_noise = r_interfere.px + (r_interfere.R + 0.05)*math.cos(r_interfere.theta)
    y2_noise = r_interfere.py + (r_interfere.R + 0.05)*math.sin(r_interfere.theta)

    # ly thuyet
    c_first.plot(r.real_tracker[0:len(r.real_tracker), 2], r.real_tracker[0:len(r.real_tracker), 3], color="red")
    c_first.plot([r.px, x2], [r.py, y2],color="black")
    c_first.plot(x_r, y_r, color="black")

    # thuc te
    c_sec.plot(r_interfere.real_tracker[0:len(r_interfere.real_tracker), 2], r_interfere.real_tracker[0:len(r_interfere.real_tracker), 3], color="green")
    c_sec.plot([r_interfere.px, x2_noise], [r_interfere.py, y2_noise],color="black")
    c_sec.plot(x_r_interfere, y_r_interfere, color="black")

    # do luong
    c_third.plot(r_interfere.measurement_tracker[0:len(r_interfere.measurement_tracker),0], r_interfere.measurement_tracker[0:len(r_interfere.measurement_tracker),1], color="yellow")

    # uoc tinh
    c_four.plot(r_interfere.estimate[0:len(r_interfere.estimate),0], r_interfere.estimate[0:len(r_interfere.estimate),1], color="purple")
    

lastPos()
plt.show()