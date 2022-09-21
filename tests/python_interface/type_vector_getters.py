
import spot

randomdistrib = spot.FISTRandomPointClouds(700, 1000, 2.0)
randomdistrib_src = randomdistrib.source_distribution

print(randomdistrib_src)