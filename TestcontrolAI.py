# force_push_scene.py
"""
Script complet intégrant :
- le monde SDF (table + cylindre) fourni,
- un robot Panda URDF (chemin à adapter si besoin),
- ContactForceEstimator (récupère forces via ContactResults),
- ForcePushController (implémentation position-based inspirée de Heins & Schoellig),
- PD+G controller,
- visualisation MeshCat, simulation et logging.

Usage:
    python force_push_scene.py
"""

import os
import math
import numpy as np
import pydot
import tempfile
from pathlib import Path

from pydrake.all import (
    DiagramBuilder, Simulator, StartMeshcat,
    AddMultibodyPlantSceneGraph, Parser,
    LogVectorOutput, ConstantVectorSource,
    BasicVector, AbstractValue,
    RigidTransform, RollPitchYaw
)
from pydrake.visualization import AddDefaultVisualization
from pydrake.systems.framework import LeafSystem
from pydrake.multibody.plant import MultibodyPlant

# ---------------- User params (change if needed) ----------------
SIM_TIME = 6.0
ROBOT_URDF_PATH = os.path.join("..", "models", "descriptions", "robots", "arms",
                               "franka_description", "urdf", "panda_arm_hand.urdf")
# Name of EE frame used by panda URDF (standard: "panda_hand")
EE_FRAME_NAME = "panda_hand"

SCENE_SDF_FILENAME = os.path.join("..", "models", "project","envtest.sdf")

# ---------------- Helpers ----------------
def normalize_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# ---------------- ContactForceEstimator ----------------
class ContactForceEstimator(LeafSystem):
    """
    Lit le port abstract 'contact_results' (connecté à plant.get_contact_results_output_port())
    et renvoie la force résultante appliquée sur le link 'object_body' (world XY).
    """
    def __init__(self, plant: MultibodyPlant, object_body):
        super().__init__()
        self.plant = plant
        self.object_body = object_body
        # Abstract input for ContactResults (type-dependent on Drake)
        # We create a placeholder AbstractValue; Drake will supply the concrete ContactResults at runtime.
        self.DeclareAbstractInputPort("contact_results", lambda: AbstractValue.Make(None))
        self.f_out = self.DeclareVectorOutputPort("force_xy", BasicVector(2), self.calc_force_out)

    def extract_contact_force_between(self, contact_results, body_index):
        """
        Generic extractor: tries several common ContactResults representations.
        Returns numpy array [fx, fy] in world frame.
        If parsing fails, returns [0,0].
        """
        fx = 0.0; fy = 0.0
        try:
            # Method 1: point_pair_contact_results (common)
            if hasattr(contact_results, "point_pair_contact_results"):
                pairs = contact_results.point_pair_contact_results()
                for pr in pairs:
                    # pr often has body_index_A, body_index_B, and force_on_A_in_world
                    try:
                        if hasattr(pr, "body_index_A") and hasattr(pr, "body_index_B"):
                            if pr.body_index_A == body_index or pr.body_index_B == body_index:
                                # read any force field available
                                if hasattr(pr, "force_on_A_in_world"):
                                    vec = np.array(pr.force_on_A_in_world).flatten()
                                elif hasattr(pr, "force"):
                                    vec = np.array(pr.force).flatten()
                                else:
                                    continue
                                fx += float(vec[0]); fy += float(vec[1])
                    except Exception:
                        continue
                return np.array([fx, fy])
            # Method 2: multibody_contact_forces (some versions)
            if hasattr(contact_results, "multibody_contact_forces"):
                mbf = contact_results.multibody_contact_forces()
                # iterate structures
                for entry in mbf:
                    try:
                        idx = entry.body_index
                        if idx == body_index:
                            vec = np.array(entry.force).flatten()
                            fx += float(vec[0]); fy += float(vec[1])
                    except Exception:
                        continue
                return np.array([fx, fy])
        except Exception as e:
            # fallback below
            print("[ContactForceEstimator] Warning parsing ContactResults:", e)

        # If nothing worked, return zero
        return np.array([0.0, 0.0])

    def calc_force_out(self, context, output):
        contact_av = self.get_input_port(0).Eval(context)
        # contact_av may be AbstractValue wrapping ContactResults or None
        # Try to get underlying value
        try:
            contact_results = contact_av.get_value() if hasattr(contact_av, "get_value") else contact_av
        except Exception:
            contact_results = contact_av

        body_idx = self.object_body.index()
        f_xy = self.extract_contact_force_between(contact_results, body_idx)
        output.SetFromVector(f_xy)

# ---------------- ForcePushController ----------------
class ForcePushController(LeafSystem):
    """
    Position-based ForcePush controller.
    Inputs:
      - robot_state (full plant state vector)
      - force_xy (2-vector)
    Output:
      - Desired_state (nq vector) : desired joint positions for PD controller
    """
    def __init__(self, plant: MultibodyPlant, ee_frame, path_fn,
                 kf=1.0, kc=0.3, ka=0.3, fmin=1.0, fmax=40.0, v_mag=0.03):
        super().__init__()
        self.plant = plant
        self.ee_frame = ee_frame
        self.path_fn = path_fn
        self.kf = kf; self.kc = kc; self.ka = ka
        self.fmin = fmin; self.fmax = fmax; self.v_mag = v_mag

        # I/O
        self.state_in = self.DeclareVectorInputPort("robot_state", BasicVector(self.plant.num_multibody_states()))
        self.force_in = self.DeclareVectorInputPort("force_xy", BasicVector(2))
        nq = self.plant.num_positions()
        self.des_out = self.DeclareVectorOutputPort("Desired_state", BasicVector(nq), self.calc_desired)
        # discrete state: previous theta
        self.DeclareDiscreteState(1)

    def calc_desired(self, context, output):
        x = self.state_in.Eval(context)
        nq = self.plant.num_positions()
        q = np.array(x[:nq])

        # plant context to compute frames, jacobians
        plant_ctx = self.plant.CreateDefaultContext()
        self.plant.SetPositions(plant_ctx, q)

        f_xy = np.array(self.force_in.Eval(context))
        f_norm = np.linalg.norm(f_xy)
        f_hat = f_xy / (f_norm + 1e-9) if f_norm > 1e-9 else np.array([1.0, 0.0])

        # EE pose
        X_WE = self.plant.CalcRelativeTransform(plant_ctx, self.plant.world_frame(), self.ee_frame)
        p_E = np.array(X_WE.translation())

        # Project to path
        p_d, t_hat = self.path_fn.project_and_tangent(p_E)
        theta_d = math.atan2(t_hat[1], t_hat[0])
        theta_f = math.atan2(f_hat[1], f_hat[0]) if f_norm > 1e-9 else theta_d
        delta_f = normalize_angle(theta_f - theta_d)
        n_hat = np.array([-t_hat[1], t_hat[0]])
        delta_c = float(np.dot(n_hat, p_E[:2] - p_d[:2]))

        theta_p = theta_d + (self.kf + 1.0)*delta_f + self.kc*delta_c

        prev_theta = context.get_discrete_state_vector().GetAtIndex(0)
        if f_norm < self.fmin:
            theta_o = theta_d - self.kc*delta_c
            theta_cmd = 0.8*prev_theta + 0.2*theta_o
        else:
            theta_cmd = theta_p

        # write back
        context.get_mutable_discrete_state_vector().SetAtIndex(0, float(theta_cmd))

        # desired cartesian position moved a bit along theta_cmd
        p_des = p_E.copy()
        p_des[0] += self.v_mag * math.cos(theta_cmd)
        p_des[1] += self.v_mag * math.sin(theta_cmd)

        # compute jacobian translational -> map delta cart to delta q
        try:
            J = self.plant.CalcJacobianTranslationalVelocity(
                plant_ctx,
                with_respect_to=self.plant.world_frame(),
                frame_B=self.ee_frame,
                p_BoBo_E=np.zeros(3),
                frame_A=self.plant.world_frame()
            )
            J_np = np.asarray(J)
            dp = p_des - p_E
            delta_q = np.linalg.pinv(J_np).dot(dp)
        except Exception as e:
            # fallback if API differs
            delta_q = np.zeros(nq)

        q_des = q + delta_q
        output.SetFromVector(q_des)

# ---------------- Simple straight-line path ----------------
class StraightLinePath:
    def __init__(self, p0=np.array([0.4, 0.0, 0.3]), dir_hat=np.array([1.0, 0.0, 0.0])):
        self.p0 = np.array(p0)
        self.dir_hat = np.array(dir_hat) / (np.linalg.norm(dir_hat) + 1e-9)
    def project_and_tangent(self, p):
        v = p[:3] - self.p0[:3]
        s = float(np.dot(v, self.dir_hat))
        p_proj = self.p0 + s*self.dir_hat
        t2 = self.dir_hat[:2]
        t2 = t2 / (np.linalg.norm(t2) + 1e-9)
        return p_proj, t2

# ---------------- Minimal PD+G controller (adapt as you want) ----------------
class Controller(LeafSystem):
    def __init__(self, plant: MultibodyPlant, Kp=None, Kd=None):
        super().__init__()
        self.plant = plant
        nq = plant.num_positions(); nv = plant.num_velocities()
        # ports
        self.state_in = self.DeclareVectorInputPort("Current_state", BasicVector(self.plant.num_multibody_states()))
        self.des_in = self.DeclareVectorInputPort("Desired_state", BasicVector(nq))
        self.tau_out = self.DeclareVectorOutputPort("tau_u", BasicVector(nv), self.calc_tau)
        # gains
        if Kp is None:
            self.Kp = np.array([120.,120.,120.,100.,50.,45.,15.,120.,120.])[:nq]
        else:
            self.Kp = np.array(Kp)[:nq]
        if Kd is None:
            self.Kd = 2.0*np.ones(nq)
        else:
            self.Kd = np.array(Kd)[:nq]

    def calc_tau(self, context, output):
        x = self.state_in.Eval(context)
        nq = self.plant.num_positions()
        q = np.array(x[:nq])
        v = np.array(x[nq:nq+self.plant.num_velocities()])
        q_des = np.array(self.des_in.Eval(context))
        e = q_des - q
        tau = self.Kp*e - self.Kd*v
        # if needed pad to nv
        if tau.shape[0] != self.plant.num_velocities():
            tau_full = np.zeros(self.plant.num_velocities())
            tau_full[:tau.shape[0]] = tau[:self.plant.num_velocities()]
            tau = tau_full
        output.SetFromVector(tau)

# ---------------- Build diagram ----------------
def create_sim_scene(sim_time_step=0.01):
    # SDF must already exist (you told me envtest.sdf is present)
    if not os.path.exists(SCENE_SDF_FILENAME):
        raise FileNotFoundError(f"SDF file not found: {SCENE_SDF_FILENAME}")

    meshcat = StartMeshcat()
    meshcat.Delete(); meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)
    # load scene (table + cylinder)
    parser.AddModelsFromUrl("file://" + os.path.abspath(SCENE_SDF_FILENAME))
    # load robot URDF if available
    if os.path.exists(ROBOT_URDF_PATH):
        parser.AddModelsFromUrl("file://" + os.path.abspath(ROBOT_URDF_PATH))
    else:
        print("[Warning] Robot URDF path not found:", ROBOT_URDF_PATH)

    plant.Finalize()

    # Identify End Effector frame
    try:
        ee_frame = plant.GetFrameByName(EE_FRAME_NAME)
        print("EE frame found:", EE_FRAME_NAME)
    except Exception:
        print("[Warning] EE frame not found. Available frames:")
        for f in plant.GetFrames():
            print("  -", f.name())
        raise RuntimeError(f"EE frame '{EE_FRAME_NAME}' not found. Change EE_FRAME_NAME accordingly.")

    # Find cylinder body robustly
    cyl_body = None
    try:
        cyl_body = plant.GetBodyByName("cylinder_link")
    except Exception:
        for b in plant.GetBodies():
            if "cylinder" in b.name().lower() or "blue_cylinder" in b.name().lower():
                cyl_body = b
                break
    if cyl_body is None:
        print("[Warning] cylinder_link not found. Bodies in plant:")
        for b in plant.GetBodies():
            print("  -", b.name())
        raise RuntimeError("Could not find the cylinder body in the plant. Check envtest.sdf names.")
    else:
        print("Cylinder body found:", cyl_body.name())

    # Add visualization
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Create controller + forcepush + contact estimator
    controller = builder.AddNamedSystem("PD+G controller", Controller(plant))
    path_fn = StraightLinePath(p0=np.array([0.45, 0.0, 0.45]), dir_hat=np.array([1.0, 0.0, 0.0]))
    forcepush = ForcePushController(plant, ee_frame, path_fn,
                                    kf=1.0, kc=0.3, ka=0.3, fmin=0.5, fmax=40.0, v_mag=0.02)
    forcepush_sys = builder.AddNamedSystem("ForcePushController", forcepush)

    contact_est = ContactForceEstimator(plant, cyl_body)
    contact_est_sys = builder.AddNamedSystem("ContactForceEstimator", contact_est)

    # Connect plant state -> controllers
    builder.Connect(plant.get_state_output_port(), controller.GetInputPort("Current_state"))
    builder.Connect(plant.get_state_output_port(), forcepush_sys.GetInputPort("robot_state"))

    # Connect contact results -> contact estimator
    try:
        builder.Connect(plant.get_contact_results_output_port(), contact_est_sys.get_input_port(0))
    except Exception as e:
        print("[Warning] Could not connect contact_results_output_port. Adapt if your Drake version differs.", e)

    # Connect estimated force -> forcepush
    builder.Connect(contact_est_sys.get_output_port("force_xy"), forcepush_sys.GetInputPort("force_xy"))

    # Connect desired from forcepush to PD+G controller
    builder.Connect(forcepush_sys.GetOutputPort("Desired_state"), controller.GetInputPort("Desired_state"))

    # Connect controller torques to plant applied generalized forces port
    builder.Connect(controller.GetOutputPort("tau_u"),
                    plant.get_applied_generalized_force_input_port())

    # Logger
    logger_state = LogVectorOutput(plant.get_state_output_port(), builder)
    logger_state.set_name("State logger")

    diagram = builder.Build()
    return diagram, logger_state, meshcat


# ---------------- Run sim ----------------
def run_simulation(sim_time_step=0.01):
    diagram, logger_state, meshcat = create_sim_scene(sim_time_step)
    sim = Simulator(diagram)
    sim.Initialize()
    sim.set_target_realtime_rate(1.0)

    # Save diagram png
    try:
        svg_data = diagram.GetGraphvizString(max_depth=2)
        graph = pydot.graph_from_dot_data(svg_data)[0]
        os.makedirs("figures", exist_ok=True)
        graph.write_png("figures/block_diagram_forcepush.png")
        print("Saved block diagram to figures/block_diagram_forcepush.png")
    except Exception as e:
        print("Could not save diagram image:", e)

    meshcat.StartRecording()
    sim.AdvanceTo(SIM_TIME)
    meshcat.PublishRecording()
    print("Simulation finished and MeshCat recording published.")

    # Simple state print
    ctx = sim.get_context()
    try:
        log = logger_state.FindLog(ctx)
        if log:
            times = log.sample_times()
            data = log.data()
            print("Log samples:", len(times))
            print("Final first joint pos:", data[0, -1])
    except Exception:
        pass

if __name__ == "__main__":
    run_simulation(sim_time_step=0.01)
