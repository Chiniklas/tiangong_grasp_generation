#!/usr/bin/env python3
import pybullet as p
import pybullet_data
import os
import time

# -----------------------------
# Config
# -----------------------------
object_urdf_dir  = "./ycb-urdfs"
# object_model     = "003_cracker_box_google_16k_textured_scale_1000.urdf"
# object_model = "005_tomato_soup_can_google_16k_textured_scale_1000.urdf"
object_model = "010_potted_meat_can_google_16k_textured_scale_1000.urdf"
object_urdf_file = os.path.join(object_urdf_dir, object_model)


# -----------------------------
# Check path
# -----------------------------
if not os.path.exists(object_urdf_file):
    raise FileNotFoundError(f"Could not find {object_urdf_file}")

print(f"[OK] Loading object URDF from: {object_urdf_file}")

# -----------------------------
# Connect to PyBullet GUI
# -----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for plane.urdf
p.setGravity(0, 0, -5.0)

# -----------------------------
# Load plane + object
# -----------------------------
plane_id = p.loadURDF("plane.urdf")
flags = (p.URDF_USE_MATERIAL_COLORS_FROM_MTL |
         p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
obj_id = p.loadURDF(object_urdf_file, basePosition=[0, 0, 0.1], flags=flags)

print("[OK] Object loaded. Inspect in the GUI window.")

# -----------------------------
# Run simulation
# -----------------------------
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
