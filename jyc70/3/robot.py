import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

class Robot:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.translation = (0,0)
        self.rotation = 0.

    def set_pose(self, pose):
        self.translation = (pose[0], pose[1])
        self.rotation = pose[2]

    def transform(self):
        trX = np.cos(self.rotation) * (self.width/2.) - np.sin(self.rotation) * (self.height/2.)
        trY = np.sin(self.rotation) * (self.width/2.) + np.cos(self.rotation) * (self.height/2.)

        tlX = np.cos(self.rotation) * -1 * (self.width / 2.) - np.sin(self.rotation) * (self.height / 2.)
        tlY = np.sin(self.rotation) * -1 * (self.width/2.) + np.cos(self.rotation) * (self.height/2.)

        tl = (tlX, tlY)
        tr = (trX, trY)
        bl = (-1 * trX, -1 * trY)
        br = (-1 * tlX, -1 * tlY)

        dtl = tuple(np.add(tl, self.translation))
        dtr = tuple(np.add(tr , self.translation))
        dbl = tuple(np.add(bl , self.translation))
        dbr = tuple(np.add(br , self.translation))

        return [dbl, dtl, dtr, dbr]

    def kinematics(self, state, control):
        xprime = control[0] * np.cos(state[2] + np.pi/2)
        yprime = control[0] * np.sin(state[2] + np.pi/2)
        return np.array([xprime, yprime, control[1]])

    def propogate(self, state, controls, durations, dt):
        if(durations==0):
            return list(state)

        final = []
        final.append(state)

        init = state
        iterations = int(durations/dt)

        for i in range(iterations):
            tempState = init
            tempControls = controls[i]

            tempKin = self.kinematics(tempState, tempControls)
            #how much to change per dt (xprime, y prime, theta prime)

            xdelta = tempKin[0] * dt
            ydelta = tempKin[1] * dt
            thetadelta = tempKin[2] * dt

            deltaState = (init[0]+xdelta ,init[1]+ydelta, init[2]+thetadelta)

            init = deltaState

            final.append(deltaState)

        return final
