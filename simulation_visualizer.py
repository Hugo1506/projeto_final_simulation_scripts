from __future__ import print_function
from gadentools.Simulation import Simulation
from gadentools.Utils import Vector3
from ipywidgets import interact, interactive, fixed, interact_manual
import ipywidgets as widgets

from gadentools.Utils import block
import numpy
import cv2
from IPython.display import clear_output

def vector3Round(vec):
  return [round(vec.x,2), round(vec.y,2), round(vec.z,2)]

sim = Simulation("/src/install/test_env/share/test_env/scenarios/new_sim_114/gas_simulations/sim1", \
                 "/src/install/test_env/share/test_env/scenarios/new_sim_114/OccupancyGrid3D.csv")

pointToQuery = Vector3(1.0, 1.0, 0.5)
iteration = 500
print("Gas Concentration at selected point:" + str( round(sim.getConcentration(iteration, pointToQuery), 2) ) + " ppm")
print("3D Wind Vector at selected point:" + str( vector3Round( sim.getWind(iteration, pointToQuery) ) ) + " m/s")