#!/usr/bin/env python
import numpy as np

class DoubleIntegrator(object):
    def __init__(self):
        # get params from server
        dims = rospy.get_param("/control_node/dims")

        self.observation_space  = np.array([[0., 0., -np.inf, -np.inf],[1., 1., np.inf, np.inf]], dtype=np.float32).T
        self.action_space       = np.array([[-1., -1.],[+1., +1.]], dtype=np.float32)
        self.explr_space        = np.array([[0., 0.],[dims[0],dims[1]]], dtype=np.float32)
        self.explr_idx          = [0, 1] # specifies which elements are used for exploration (i.e x,y)

        self.dt = 0.1

        self._A = np.array([
                [0., 0., 0.8, 0.], # x
                [0., 0., 0., 0.8], # y
                [0., 0., 0., 0.],  # x_dot
                [0., 0., 0., 0.]   # y_dot
        ])

        self._B = np.array([
                [0., 0.],
                [0., 0.],
                [1.0, 0.],
                [0., 1.0]
        ])

        self.reset()

    def reset(self, state=None):
        '''
        Resets the property self.state
        '''
        if state is None:
            self.state = np.random.uniform(0.1, 0.2, size=(self.observation_space.shape[0],))
        else:
            self.state = state.copy()

        return self.state.copy()

    @property
    def state(self):
        return self._state.copy()
    @state.setter
    def state(self, x):
        self._state = x.copy()
    @property
    def A(self, x=None, u=None):
        return self._A.copy()
    @property
    def B(self, x=None, u=None):
        return self._B.copy()

    def f(self, x, u):
        '''
        Continuous time dynamics
        '''
        print('self._A:')
        print(self._A)
        print('x: ')
        print(x)
        print('self._B')
        print(self._B)
        print('u: ')
        print(u)
        return np.dot(self._A, x) + np.dot(self._B, u)

    def step(self, a): # this is where `super` from agent.py goes
        state = self._state + self.f(self._state, a) * self.dt
        self._state = state
        return state.copy()
