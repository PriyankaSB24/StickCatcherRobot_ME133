'''stickpredict.py
        * Predict where and when a falling stick can be caught.
        * All positions and velocities are in the world frame.
        * This file contains only math, no ROS.

    The functions do the following: 
        * predict(p, v, tau) --> (pd, vd)
            - predict the position & velocity after tau seconds
        * intercept(p, v) --> (pc, tau)
            - find the time & position at which the stick reaches the catching height [zcatch]
        * feasible_intercept(p, v, phand) --> (pc, tau)
            - find a catch point that the hand has enough time to reach
'''

import numpy as np
from math import sqrt

G = 9.81
ZCATCH = 1.05 # fixed catching height in meters

# predict the position and velocity after tau seconds.
def predict(p, v, tau, g=G):
    p = np.asarray(p).reshape(3)
    v = np.asarray(v).reshape(3)

    a = np.array([0.0, 0.0, -g]) # acceleration due to gravity  in [x, y, z]
    pd = p + v * tau + 0.5 * a * tau**2 # position after tau seconds
    vd = v + a * tau # velocity after tau seconds

    return pd, vd


# find the time and position at which the stick reaches zcatch
def intercept(p, v, zcatch=ZCATCH, g=G):
    p = np.asarray(p).reshape(3)
    v = np.asarray(v).reshape(3)

    # if the stick has already passed [below] the catching height
    if p[2] <= zcatch:
        return None, None

    # otherwise solve the quadratic equation for the falling time [tau]
    tau = (v[2] + sqrt(v[2]**2 + 2.0 * g * (p[2] - zcatch))) / g
    pc, _ = predict(p, v, tau, g)

    return pc, tau


# find a catch point that the hand has enough time to reach
def feasible_intercept(p, v, phand, vmax=3.0, zmin=0.9, zmax=1.4, g=G):
    phand = np.asarray(phand).reshape(3) # hand position in world frame

    # try higher catch points first, then lower ones.
    for zcatch in np.linspace(zmax, zmin, 26):

        pc, tau = intercept(p, v, zcatch, g)

        if pc is None: # the stick has already passed below the catching height
            continue

        distance = np.linalg.norm(pc - phand)

        # a quintic spline has a peak speed of 1.875*d/T from previous sets 
        # [and also my cheat sheet TEEHEE]
        treach = 1.875 * distance / vmax

        # allow at least 0.1 seconds to react
        if tau >= max(treach, 0.10): 
            return pc, tau

    # no suitable catch point was found.
    return None, None


# simple test, without launching ROS --> Run: 
#    cd ~/robotws/src/stickcatcher/stickcatcher
#    python3 stickpredict.py

if __name__ == '__main__':

    p = np.array([0.45, -0.30, 2.20])
    v = np.zeros(3)

    pc, tau = intercept(p, v)

    print("Catch position:", pc)
    print("Time remaining:", tau)