from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np

# ------------------------------------------------------------------------
def sample(x):
    return random.gauss(0.0, x)

def sample_motion(particle, odom):
    (xbar, ybar, tbar) = odom[0]
    (xbarprime, ybarprime, tbarprime) = odom[1]

    drot1 = np.degrees(np.arctan2(ybarprime - ybar, xbarprime - xbar)) - tbar
    dtrans = np.sqrt((xbar - xbarprime) ** 2 + (ybar - ybarprime) ** 2)
    drot2 = tbarprime - tbar - drot1
    
    hdrot1 = drot1 - sample(ODOM_HEAD_SIGMA)
    hdtrans = dtrans - sample(ODOM_TRANS_SIGMA)
    hdrot2 = drot2 - sample(ODOM_HEAD_SIGMA)

    xprime = particle.x + hdtrans * np.cos(np.radians(particle.h + hdrot1))
    yprime = particle.y + hdtrans * np.sin(np.radians(particle.h + hdrot1))
    tprime = particle.h + hdrot1 + hdrot2

    return Particle(xprime, yprime, tprime)

def motion_update(particles, odom):
    for i in range(len(particles)):
        particles[i] = sample_motion(particles[i], odom)
    return particles

# ------------------------------------------------------------------------
def gaussian_prob(mu, sigma, x):
    return (1/(sigma * np.sqrt(2*np.pi))) * (np.e ** ((-1/2) * (((x - mu)/sigma) ** 2)))

def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map, which contains the marker information, 
                see grid.h and CozGrid for definition

        Returns: the list of particle represents belief p(x_{t} | u_{t})
                after measurement update
    """
    probabilities = []
    for particle in particles:
        particle_simulated_seen = particle.read_markers(grid)
        if len(particle_simulated_seen) != len(measured_marker_list):
            probabilities.append(0)
        else:
            compound_prob = 1
            for simulated in particle_simulated_seen: # currently doing nothing to match up "best fit" pairs of markers between particle and agent. Code could improve with this.
                    rhat = np.sqrt((simulated[0] ** 2) + (simulated[1] ** 2))
                    phihat = np.arctan2(simulated[0], simulated[1])
                    for marker in measured_marker_list:
                        r = np.sqrt((marker[0] ** 2) + (marker[1] ** 2))
                        phi = np.arctan2(marker[0], marker[1])
                        compound_prob *= gaussian_prob(0, MARKER_TRANS_SIGMA, r - rhat) * gaussian_prob(0, MARKER_ROT_SIGMA, phi - phihat)
            probabilities.append(compound_prob)
    s = sum(probabilities)
    if s == 0: # if sum of probabilities is zero, none of the particles saw the correct # of markers.
        return particles # Move them blindly onward in hopes measurements get better soon.
    probabilities[:] = [x / s for x in probabilities]
    return np.random.choice(particles, 5000, p=probabilities)
