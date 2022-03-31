#!/usr/bin/env python
import numpy as np

class Barrier(object):
    """This class prevents the agent from going outside the exploration space."""
    def __init__(self, explr_space, pow=2, weight=100): #original pow value was 2. I'm trying pow=6 to make stronger barrier functions
        self.explr_space = explr_space
        self.dl = explr_space[1,:] - explr_space[0,:]
        self.pow = pow
        self.weight = weight
        self.eps = 0.1

    def cost(self, x):
        """Returns the actual cost of the barrier."""
        cost = 0.
        cost += np.sum((x > self.explr_space[1,:]-self.eps) * (x - (self.explr_space[1,:]-self.eps))**self.pow)
        cost += np.sum((x < self.explr_space[0,:]+self.eps) * (x - (self.explr_space[0,:]+self.eps))**self.pow)
        return self.weight * cost

    def dx(self, x):
        """Returns the derivative of the barrier w.r.t the exploration state."""
        dx = np.zeros(x.shape)
        dx += 2*(x > (self.explr_space[1,:]-self.eps)) * (x - (self.explr_space[1,:]-self.eps))
        dx += 2*(x < (self.explr_space[0,:]+self.eps)) * (x - (self.explr_space[0,:]+self.eps))

        # creating stronger barrier functions: pow=6, raising (x - (self.explr_space+-self.eps)) to power (self.pow-1); using coefficient=6 instead of 2 because it's a derivative (x^6 -> 6x^5)
        #dx += 6*(x > (self.explr_space[1,:]-self.eps)) * (x - (self.explr_space[1,:]-self.eps))**(self.pow-1)
        #dx += 6*(x < (self.explr_space[0,:]+self.eps)) * (x - (self.explr_space[0,:]+self.eps))**(self.pow-1)

        # breaking dx down into components since the space is not square-shaped
        #dx[0] += 6*(x[0] > (self.explr_space[1,0] - self.eps*1)) * (x[0] - (self.explr_space[1,0] - self.eps*1) + 1)**5
        #dx[1] += 6*(x[1]*3.5 > (self.explr_space[1,1] - self.eps*3.5)) * (x[1]*3.5 - (self.explr_space[1,1] - self.eps*3.5) + 1)**5

        #dx[0] += 6*(x[0] < (self.explr_space[0,0] + self.eps*1)) * (x[0] - (self.explr_space[0,0] + self.eps*1) + 1)**5
        #dx[1] += 6*(x[1]*3.5 < (self.explr_space[0,1] + self.eps*3.5)) * (x[1]*3.5 - (self.explr_space[0,1] + self.eps*3.5) + 1)**5
        return self.weight * dx
