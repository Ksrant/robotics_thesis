from pydrake.all import DiagramBuilder, AddMultibodyPlantSceneGraph, Parser
import os

SDF = "/Users/Antoine/Documents/thesis/Robotics-II-main/models/project/envtest.sdf"
URDF = "/Users/Antoine/Documents/thesis/Robotics-II-main/models/descriptions/robots/arms/franka_description/urdf/panda_arm_hand.urdf"

builder = DiagramBuilder()
time_step = 0.001
plant, scene_graph = AddMultibodyPlantSceneGraph(builder,time_step)
parser = Parser(plant)

# Load models
parser.AddModelsFromUrl("file://" + os.path.abspath(SDF))
parser.AddModelsFromUrl("file://" + os.path.abspath(URDF))

# Finalize plant
plant.Finalize()
print("Loaded SDF + URDF successfully. Bodies:")
for b in plant.GetBodies():
    print(" -", b.name())
