# Imports SPOT, and checks the existence of types

import spot

spot_random = spot.FISTRandomPointClouds(700, 1000, 1.0)
#spot_single = spot.FISTSamePointClouds("")
#spot_models = spot.FISTDifferentPointClouds("", "")

spot_glm_vec3 = spot.glm_vec3()
spot_glm_vec4 = spot.glm_vec4()
spot_glm_mat3 = spot.glm_mat3()
spot_glm_mat4 = spot.glm_mat4()

print(spot_glm_mat4)
print(spot_glm_mat3)
print(spot_glm_vec4)
print(spot_glm_vec3)

spot_Point3f = spot.Point3f()
spot_Point3d = spot.Point3d()
