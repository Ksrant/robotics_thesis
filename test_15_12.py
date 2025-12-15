import numpy as np
import time
import os
from pydrake.all import (
    DiagramBuilder,
    MultibodyPlant,
    SceneGraph,
    Parser,
    Simulator,
    RigidTransform,
    SpatialForce,
    LeafSystem,
    BasicVector,
    AbstractValue,
    Meshcat,
    MeshcatVisualizer,
    StartMeshcat
)

from pydrake.multibody.plant import ContactResults
from pydrake.math import RollPitchYaw


robot_path = os.path.join(
    "..", "models", "descriptions", "robots", "arms", "franka_description", "urdf", "panda_arm_hand.urdf"
)
sdf_path = os.path.join(
    "..", "models", "project", "envtest.sdf"
)
class ContactForceEstimator(LeafSystem):
    def __init__(self, plant: MultibodyPlant, body):
        super().__init__()
        self.plant = plant
        self.body = body

        self.DeclareAbstractInputPort(
            "contact_results",
            AbstractValue.Make(ContactResults())
        )

        self.force_out = self.DeclareVectorOutputPort(
            "force_xy", BasicVector(2), self.CalcForce
        )

    def CalcForce(self, context, output):
        contact_results = self.get_input_port(0).Eval(context)

        fx, fy = 0.0, 0.0

        for i in range(contact_results.num_point_pair_contacts()):
            info = contact_results.point_pair_contact_info(i)

            if (
                info.bodyA_index() == self.body.index()
                or info.bodyB_index() == self.body.index()
            ):
                f = info.contact_force()
                fx += f[0]
                fy += f[1]

        output.SetFromVector([fx, fy])

class ForcePushSystem(LeafSystem):
    def __init__(self):
        super().__init__()
        self.DeclareVectorInputPort("force_xy", 2)
        self.DeclareVectorOutputPort("dummy", 1, self.DoNothing)

    def DoNothing(self, context, output):
        output.SetAtIndex(0, 0.0)

def create_sim_scene(time_step=0.001):
    builder = DiagramBuilder()

    plant, scene_graph = MultibodyPlant(time_step), SceneGraph()
    builder.AddSystem(plant)
    builder.AddSystem(scene_graph)
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    parser = Parser(plant)

    # Panda
    parser.AddModelsFromUrl("file://" + os.path.abspath(robot_path))

    parser.AddModelsFromUrl("file://" + os.path.abspath(sdf_path))

    builder.Connect(
        plant.get_geometry_pose_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id())
    )
    builder.Connect(
        scene_graph.get_query_output_port(),
        plant.get_geometry_query_input_port()
    )

    # Meshcat
    meshcat = StartMeshcat()
    viz = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Bodies
    model = plant.GetModelInstanceByName("blue_cylinder")
    cyl_body  = plant.GetBodyByName("cylinder_link", model)
    plant.Finalize()
    # Contact estimator
    contact_est = builder.AddSystem(ContactForceEstimator(plant, cyl_body))
    builder.Connect(
        plant.get_contact_results_output_port(),
        contact_est.get_input_port(0)
    )

    # Dummy force push system
    forcepush = builder.AddSystem(ForcePushSystem())
    builder.Connect(
        contact_est.force_out,
        forcepush.GetInputPort("force_xy")
    )

    diagram = builder.Build()
    return diagram, plant, Meshcat

def run_simulation(sim_time_step=0.001):
    diagram, plant, meshcat = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    plant_context = plant.GetMyContextFromRoot(context)

    # Position initiale cylindre
    plant.SetFreeBodyPose(
        plant_context,
        plant.GetBodyByName("link", plant.GetModelInstanceByName("cylinder")),
        RigidTransform([0.5, 0, 0.15])
    )

    simulator.Initialize()
    simulator.AdvanceTo(5.0)

    print("Simulation finished")

if __name__ == "__main__":
    run_simulation()

