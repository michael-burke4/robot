from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np
import time

def sample(x):
    return random.gauss(0.0, x)
# ------------------------------------------------------------------------
def sample_motion(particle, odom, a1, a2, a3, a4):
    (xbar, ybar, tbar) = odom[0]
    (xbarprime, ybarprime, tbarprime) = odom[1]

    drot1 = np.arctan2(ybarprime - ybar, xbarprime - xbar) - tbar
    dtrans = np.sqrt((xbar - xbarprime) ** 2 + (ybar - ybarprime) ** 2)
    drot2 = tbarprime - tbar - drot1
    
    hdrot1 = drot1 - sample(a1*drot1 + a2*dtrans)
    hdtrans = dtrans - sample(a3 * dtrans + (a4 * (drot1 + drot2)))
    hdrot2 = drot1 - sample(a1*drot1 + a2*dtrans)

    xprime = particle.x + hdtrans * np.cos(particle.h + hdrot1)
    yprime = particle.y + hdtrans * np.sin(particle.h + hdrot1)
    tprime = particle.h + hdrot1 + hdrot2

    return Particle(xprime, yprime, tprime)

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- noisy odometry measurements, a pair of robot pose, i.e. last time
                step pose and current time step pose

        Returns: the list of particle representing belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    alpha1 = 0.001
    alpha2 = 0.001
    alpha3 = 0.005
    alpha4 = 0.005
    #print("particles:", particles)
    for i in range(len(particles)):
        particles[i] = sample_motion(particles[i], odom, alpha1, alpha2, alpha3, alpha4)

    
    return particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- a list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map containing the marker information. 
                see grid.py and CozGrid for definition

        Returns: the list of particle representing belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = []
    #return measured_particles
    return particles
