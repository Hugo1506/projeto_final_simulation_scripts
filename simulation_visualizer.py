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

# ---- Simulation Setup ----
sim = Simulation("/src/install/test_env/share/test_env/scenarios/new_sim_114/gas_simulations/sim1", \
                 "/src/install/test_env/share/test_env/scenarios/new_sim_114/OccupancyGrid3D.csv")

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

    # Save frames as GIF
    gif_path = 'test_'+str(height)+'.gif'
    frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0)
    print(f"\n✅ GIF saved at: {os.path.abspath(gif_path)}")

# ---- Run It! ----

for height in numpy.arange(minHeight, maxHeight, 0.5):
  run_simulation_and_save_gif(height)
