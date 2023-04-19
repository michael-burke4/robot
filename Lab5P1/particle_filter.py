from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np

# ------------------------------------------------------------------------
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
    new_particles = []
    for i in range(len(particles)):
        xbar, ybar, tbar = odom[0]
        xbarprime, ybarprime, tbarprime = odom[1]

        drot1 = np.arctan2(ybarprime - ybar, xbarprime - xbar) - tbar
        dtrans = np.sqrt((xbar - xbarprime) ** 2 + (ybar - ybarprime) ** 2)
        drot2 = tbarprime - tbar - drot1

        hdrot1 = drot1 - random.gauss(0.0, (alpha1 * drot1 + alpha2 * dtrans))
        hdtrans = dtrans - random.gauss(0.0, (alpha3 * dtrans + (alpha4 * (drot1 + drot2))))
        hdrot2 = drot2 - random.gauss(0.0, alpha1 * drot1 + alpha2 * dtrans)

        xprime = particles[i].x + hdtrans * np.cos(particles[i].h + hdrot1)
        yprime = particles[i].y + hdtrans * np.sin(particles[i].h + hdrot1)
        tprime = particles[i].h + hdrot1 + hdrot2

        particles[i] = Particle(xprime, yprime, tprime)
        
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
    for particle in particles:
        weight = 1
        for marker in measured_marker_list:
            # Convert marker's relative coordinates and heading to world coordinates and heading
            #
            #
            # Get the marker value in the grid
            marker_value = grid.random_place()
            # If the marker is not in the grid, set the marker value to 0
            if marker_value is None:
                marker_value = 0
            # Calculate the weight of the particle based on the marker value
            #
            #
        # Create a new particle with the same pose but updated weight
        measured_particles.append(Particle(particle.x, particle.y, particle.h, weight))
    # Normalize the weights of the particles
    total_weight = sum([particle.weight for particle in measured_particles])
    for particle in measured_particles:
        particle.weight /= total_weight
    return measured_particles