import os
import pybullet as p
import json
import math
import numpy as np
import imageio

from grasp_pc import set_pose_dofs

dataset_dir = "/root/data/output_dataset"

initial_dir = "initial_grasps"

object_model_name = "cylinder"

gripper_name = "handright9253"

grasps_fpath = os.path.join(
    dataset_dir,
    initial_dir, f"{object_model_name}-{gripper_name}.json")

urdf_dir = '../prepare-graspit-urdf/urdf/'


# images_dir = './images/' + gripper_name
images_dir = os.path.join(dataset_dir, object_model_name, "images", gripper_name)

if not os.path.isdir(images_dir):
    os.makedirs(images_dir)

# Camera related stuff
width = 400
height = 400
fov = 60
aspect = width / height
near = 0.02
far = 1

view_matrix = p.computeViewMatrix([0, 0, 0.5], [0, 0, 0], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

cam_target_pos = [-0.97, 0.21, -0.33]
distance = 1.80
cam_pitch = -23.80
cam_yaw = 74.0
cam_roll = 0
up_axis = 2 # 1 for Y-axis, 2 for Z-axis

view_matrix_2 = p.computeViewMatrixFromYawPitchRoll(
    cam_target_pos,
    distance,
    cam_yaw,
    cam_pitch,
    cam_roll,
    2)

# Gripper URDF Loading part

if gripper_name == "handright9253":
    gripper_urdf = "handright9253/handright9253.urdf"

gripper_urdf_file = os.path.join(urdf_dir, gripper_urdf)

# connect to PyBullet GUI
p.connect(p.GUI) # with GUI, useful to see final result

grasps_fpath = "../../output_dataset/initial_grasps/cylinder-handright9253.json"
with open(grasps_fpath, "r") as gf:
    grasp_data = json.load(gf)

obj_id = p.loadURDF("./ycb-urdfs/010_potted_meat_can_google_16k_textured_scale_1000.urdf")

grasps = grasp_data['grasps']
robo_id = p.loadURDF(gripper_urdf_file)
for idx, info in enumerate(grasps):
    full_pose = info['pose']
    pos = full_pose[:3]
    orn = full_pose[3:]
    dofs = info['dofs']
    # tmp = p.loadURDF(gripper_urdf_file, basePosition=pos, baseOrientation=orn)
    set_pose_dofs(gripper_name, robo_id, full_pose, dofs)
    # w,h,img_rgb,img_dep,img_seg = p.getCameraImage(width, height, view_matrix_2, projection_matrix)
    # img = np.reshape(img_rgb, (w,h,4))
    img_fname = f'{gripper_name}-{object_model_name[:-20]}-refined_gnum_{idx}.png'
    # imageio.imwrite(os.path.join(images_dir, img_fname), img)
    # img_fname = f'img_graspnum_{idx}.png'

p.isNumpyEnabled()

test = 'ninad\
adad'