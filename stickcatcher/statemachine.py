'''
states:
wait: sits at home pose until catchable stick is available
step_reach: steps off the home pose and reaches hand to intercept
grab: holds stick momentarily 
reset: returns to home base, starts process over 

catch point and time will be pulled from stickpredict.feasible_intercept()
which returns a point where the hand has enough time to reach
    - if there is a feasible intercept, go 
    - catch.py calls feasible_intercept(), passes result
    - catch.py will call update() every cycle and builds task trajectory based on the current state
'''
import numpy as np

#state names
WAIT = 'WAIT'
STEP_REACH = 'STEP_REACH'
GRAB = 'GRAB'
RESET = 'RESET'

class StateMachine:
    # initialization
    def __init__(self, tol=0.03, tlate=0.2, tgrab=0.2, treset=3.0):
        self.tol = tol      #distance between the hand to stick (guessed for now because dont have sizes)
        self.tlate = tlate  #time past thit to give up
        self.tgrab = tgrab  #seconds holding onto stick (estimate)
        self.treset = treset    #time it takes to come back home

        #initializing state and intercept 
        self.state = WAIT
        self.tenter = 0.0   #time when the state begins
        self.thit = None    #interception time
        self.phit = None    #interception position
        self.caught = False #if the stick was caught or not 
        self.newstick = False   #indicates expectation for next stick

    #indicates time since entering the current state
    def elapsed(self, t):
        return t - self.tenter

    #indicates the fraction of how far off it is from getting the stick
    def reachFrac(self, t):
        T = max(self.thit - self.tenter, 1e-6)
        return min(max((t - self.tenter) / T, 0.0), 1.0)

    #checks new stick request
    def checknewstick(self):
        new = self.newstick
        self.newstick = False
        return new

    def goto(self, t, state):
        self.state = state
        self.tenter = t

    '''
    update calls every cycle 
    t: the current time
    tip_p: current position of hand
    stick_p: current position of the stick
    pc, tau: catch point and time, returned by feasible_intercept(p, v, phand)
                returns none for both if there is no stick or no catch 
    '''
    def update(self, t, tip_p, stick_p, pc, tau):
        if self.state == WAIT:
            if (stick_p is not None) and (pc is not None):
                self.phit = np.array(pc)
                self.thit = t + tau
                self.goto(t, STEP_REACH)

        elif self.state == STEP_REACH:
            #defines caught stick, missed, or to continue
            if stick_p is None:
                self.caught = False
                self.goto(t, RESET)
            elif np.linalg.norm(tip_p - stick_p) < self.tol:
                self.caught = True
                self.goto(t, GRAB)
            elif t > self.thit + self.tlate:
                self.caught = False
                self.goto(t, RESET)

        elif self.state == GRAB:
            if self.elapsed(t) >= self.tgrab:
                self.goto(t, RESET)

        else:
            if self.elapsed(t) >= self.treset:
                self.newstick = True 
                self.goto(t, WAIT)

        return self.state