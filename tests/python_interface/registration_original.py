# Registers using the FIST method with the same parameters as the original mainFIST.cpp program.

import numpy as np
import spot

spot.enable_reproducible_runs()
r = spot.FISTRandomPointClouds(700, 1000, 1.0)
r.compute_transformation(True)
print(np.array(r.matrix()))
print(np.array(r.translation()))

exit(1)