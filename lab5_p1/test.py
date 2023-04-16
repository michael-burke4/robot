from particle import *
from particle_filter import *

alpha1 = 0.001
alpha2 = 0.001
alpha3 = 0.005
alpha4 = 0.005
p = Particle(0, 0, 0)
print(p)
p = sample_motion(p, [[0, 0, 0], [10, 0, 0]], alpha1, alpha2, alpha3, alpha4)
print(p)
p = sample_motion(p, [[10, 0, 0], [20, 0, 0]], alpha1, alpha2, alpha3, alpha4)
print(p)
p = sample_motion(p, [[20, 0, 0], [30, 0, 0]], alpha1, alpha2, alpha3, alpha4)
print(p)