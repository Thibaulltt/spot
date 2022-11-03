# Imports SPOT, and checks the existence of types
import os, sys
# Include local build path !
sys.path.insert(0, '../../build/release')

import spot
import pdb

spot_random = spot.FISTRandomPointClouds(700, 1000, 1.0)
#spot_single = spot.FISTSamePointClouds("")
#spot_models = spot.FISTDifferentPointClouds("", "")

spot_vec3 = spot.vec3()
spot_vec4 = spot.vec4()
spot_mat3 = spot.mat3()
spot_mat4 = spot.mat4()

print(spot_mat4)
print(spot_mat3)
print(spot_vec4)
print(spot_vec3)

spot_Point3f = spot.Point3f()
spot_Point3d = spot.Point3d()

print(spot_Point3f)
print(spot_Point3d)

spot_Timings = spot.TimingsLogger(12)

spot_Timings.print_timings()
