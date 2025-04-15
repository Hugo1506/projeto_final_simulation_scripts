from __future__ import print_function
from gadentools.Simulation import Simulation
from gadentools.Utils import Vector3
from gadentools.Utils import block
import numpy
import cv2
from PIL import Image
import imageio
import time
import os
from IPython.display import clear_output
import sys
import requests
import base64
import io
import zlib

scenario_path = os.path.join("/src/install/test_env/share/test_env/scenarios",sys.argv[1])
simulation_path = os.path.join(scenario_path,"gas_simulations/sim1")
ocuppancy_path = os.path.join(scenario_path,"OccupancyGrid3D.csv")

simulation_name = sys.argv[1].replace("_sim_", "_")

# ---- Simulation Setup ----
sim = Simulation(simulation_path, \
                 ocuppancy_path)

# ---- Parameters ----
maxHeight = sim.env_max.z
timeLimitSeconds = 20        
minHeight = sim.env_min.z              
max_ppm = 20.0                 
imageSizeFactor = 5          
frame_duration_ms = 50     

# ---- Store frames here ----
frames = []

# ---- Main Simulation Loop & GIF Capture ----
def run_simulation_and_save_gif(height):
    frames.clear()
    sim.playSimulation(0, 0.5)
    time_start = time.time()
    gif_io = io.BytesIO()

    while (time.time() - time_start) < timeLimitSeconds:
        clear_output(wait=True)
        iteration = sim.getCurrentIteration()

        # Generate concentration map
        map = sim.generateConcentrationMap2D(iteration, height, True)
        map_scaled = map * (255.0 / max_ppm)
        formatted_map = numpy.array(numpy.clip(map_scaled, 0, 255), dtype=numpy.uint8)

        # Apply colormap
        heatmap = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)

        # Draw obstacles in black
        block(map, heatmap)

        # Resize and rotate for display
        newshape = (imageSizeFactor * heatmap.shape[1], imageSizeFactor * heatmap.shape[0])
        heatmap = cv2.resize(heatmap, newshape)
        heatmap = cv2.rotate(heatmap, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Convert to RGB for saving
        rgb_image = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)
        frames.append(pil_img)

        print(f"Captured frame for iteration {iteration}")
        time.sleep(0.5)

    sim.stopPlaying()

    gif_io = io.BytesIO()
    frames[0].save(gif_io, format='GIF', save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0)

    gif_raw = gif_io.getvalue()

    compressed_gif = zlib.compress(gif_raw)

    compressed_gif_base64 = base64.b64encode(compressed_gif).decode('utf-8')

    response = requests.post('http://172.17.0.3:3000/uploadSimulationResults', json={
        'simulation': simulation_name,
        'type': 'heatmap',
        'gif': compressed_gif_base64,
        'height': height,
    })
    

# ---- Run It! ----

for height in numpy.arange(minHeight, maxHeight, 0.5):
  run_simulation_and_save_gif(height)
