import os
import numpy as np
import pydot
import matplotlib.pyplot as plt
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.primitives import ConstantVectorSource, LogVectorOutput
from pydrake.multibody.tree import JacobianWrtVariable, ModelInstanceIndex
from pydrake.visualization import AddDefaultVisualization
from helper.dynamics import CalcRobotDynamics

meshcat = StartMeshcat()
meshcat.Delete()
meshcat.DeleteAddedControls()

ROBOT_URDF_PATH    = os.path.join("..", "models", "descriptions", "robots", "arms",
                                   "franka_description", "urdf", "panda_arm_hand_sphere.urdf")
SCENE_SDF_FILENAME = os.path.join("..", "models", "descriptions", "envcube.sdf")

# ═══════════════════════════════════════════════════════════════════════════════
#  GÉOMÉTRIE
# ═══════════════════════════════════════════════════════════════════════════════
FINGER_TIP_OFFSET = np.array([0.0, 0.0, 0.0])
SPHERE_RADIUS     = 0.05
CUBE_POS          = np.array([0.35, 0.25, 0.075])
CUBE_HALF         = 0.05
TABLE_TOP_Z       = 0.050
Z_PUSH            = TABLE_TOP_Z + SPHERE_RADIUS   # 0.10 m
Z_FLOOR           = Z_PUSH - 0.003               # 0.097 m

ROBOT_BASE_OFFSET = RigidTransform(RotationMatrix.Identity(), [-0.10, 0.0, 0.0])
CUBE_TARGET       = np.array([0.25, 0.0])

# ═══════════════════════════════════════════════════════════════════════════════
#  PARAMÈTRES
# ═══════════════════════════════════════════════════════════════════════════════
V_PUSH    = 0.04
K_CONTACT = 200.0
F_MIN     = 0.05
F_MAX     = 15.0
BETA      = 0.30
K_F       = 0.15
K_C       = 0.10
K_LAT     = 1.5

Kp_pos = 400.0
Kd_pos = 80.0
APPROACH_Z         = 0.35
WAYPOINT_THRESHOLD = 0.020   # m

# FIX #1 : seuil DONE relevé à 0.020m — 5mm était inaccessible en pratique
# (le cube oscille légèrement autour de la cible et n'atteint jamais 5mm exactement)
DONE_THRESHOLD     = 0.01   # m (était 0.005 — trop serré)

NO_CONTACT_TIMEOUT = 3.0     # s
RETRAIT            = 0.10    # m (augmenté de 0.08 → 0.10 pour plus de marge)

print(f"[Config] Z_PUSH={Z_PUSH}m  V_PUSH={V_PUSH}m/s  K_LAT={K_LAT}  target={CUBE_TARGET}")
print(f"[Config] DONE_THRESHOLD={DONE_THRESHOLD}m")


# ═══════════════════════════════════════════════════════════════════════════════
#  UTILITAIRES GÉOMÉTRIQUES
# ═══════════════════════════════════════════════════════════════════════════════

def best_face_normal(push_dir_xy):
    candidates = np.array([[1,0],[-1,0],[0,1],[0,-1]], float)
    return candidates[np.argmin(candidates @ push_dir_xy)]


def contact_point_xy(cube_pos_xy, face_normal_xy):
    return cube_pos_xy + (CUBE_HALF + SPHERE_RADIUS - 0.005) * face_normal_xy


def hover_point_xy(cube_pos_xy, face_normal_xy):
    return cube_pos_xy + (CUBE_HALF + SPHERE_RADIUS + 0.12) * face_normal_xy


def estimate_contact_force(p_sph_xy, push_dir_xy, cube_pos_xy):
    candidates = np.array([[1,0],[-1,0],[0,1],[0,-1]], float)
    diff   = p_sph_xy - cube_pos_xy
    f_norm = candidates[np.argmax(candidates @ diff)]
    face_c = cube_pos_xy + CUBE_HALF * f_norm
    d      = float((p_sph_xy - face_c) @ f_norm)
    pen    = float(np.clip(SPHERE_RADIUS - d, 0.0, 0.02))
    if pen < 1e-5:
        return np.zeros(2)
    return K_CONTACT * pen * push_dir_xy


# ═══════════════════════════════════════════════════════════════════════════════
#  ÉTATS
# ═══════════════════════════════════════════════════════════════════════════════
STATE_APPROACH   = "APPROACH"
STATE_PUSH       = "PUSH"
STATE_REPOSITION = "REPOSITION"
STATE_DONE       = "DONE"


# ═══════════════════════════════════════════════════════════════════════════════
#  CONTRÔLEUR
# ═══════════════════════════════════════════════════════════════════════════════
class PushController(LeafSystem):
    def __init__(self, plant, init_q7):
        super().__init__()
        nq = plant.num_positions()
        nv = plant.num_velocities()
        self._state_port   = self.DeclareVectorInputPort("Current_state", size=nq+nv)
        self._desired_port = self.DeclareVectorInputPort("Desired_state",  size=7)

        self.plant            = plant
        self.plant_context_ad = plant.CreateDefaultContext()
        self._nq, self._nv    = nq, nv

        self._panda_model = plant.GetModelInstanceByName("panda")
        vel_idx = []
        for ji in plant.GetJointIndices(self._panda_model):
            jt = plant.get_joint(ji)
            if jt.num_velocities() > 0:
                vel_idx.extend(range(jt.velocity_start(),
                                     jt.velocity_start() + jt.num_velocities()))
        self._arm_vel_idx = np.array(vel_idx, dtype=int)

        self._cube_model = plant.GetModelInstanceByName("cube")
        cube_idx = []
        for ji in plant.GetJointIndices(self._cube_model):
            jt = plant.get_joint(ji)
            if jt.num_positions() > 0:
                cube_idx.extend(range(jt.position_start(),
                                      jt.position_start() + jt.num_positions()))
        self._cube_pos_idx = np.array(cube_idx, dtype=int)

        self._ctrl_state   = STATE_APPROACH
        self._waypoints    = []
        self._wp_idx       = 0
        self._f_filt       = np.zeros(2)
        self._theta_ee     = 0.0
        self._contact      = False
        self._no_contact_t = 0.0
        self._push_dir     = np.zeros(2)
        self._face_normal  = np.zeros(2)

        plant.SetPositions(self.plant_context_ad, self._panda_model, np.array(init_q7))
        p0, _ = self._get_sphere_center()
        print(f"[Controller] sphere init={np.round(p0,4)}")
        self._plan_approach(CUBE_POS[:2])

        state_idx = self.DeclareDiscreteState(nv)
        self.DeclareStateOutputPort("tau_u", state_idx)
        self.DeclarePeriodicDiscreteUpdateEvent(1/1000, 0.0, self._update)
        self.DeclarePeriodicPublishEvent(1, 0, self._log)

    # ── Cinématique ──────────────────────────────────────────────────────────

    def _get_sphere_center(self):
        ee   = self.plant.GetFrameByName("panda_hand")
        X_WE = self.plant.CalcRelativeTransform(
            self.plant_context_ad, self.plant.world_frame(), ee)
        R    = X_WE.rotation()
        return X_WE.translation() + R.matrix() @ FINGER_TIP_OFFSET, R

    def _get_jacobians(self):
        ee     = self.plant.GetFrameByName("panda_hand")
        J_full = self.plant.CalcJacobianSpatialVelocity(
            self.plant_context_ad, JacobianWrtVariable.kV,
            ee, FINGER_TIP_OFFSET,
            self.plant.world_frame(), self.plant.world_frame())
        J = J_full[:, self._arm_vel_idx]
        return J[:3, :], J[3:, :]

    def _get_cube_xy(self, q):
        if len(self._cube_pos_idx) == 7:
            return q[self._cube_pos_idx[4:7]][:2].copy()
        return CUBE_POS[:2].copy()

    # ── Planification ────────────────────────────────────────────────────────

    def _plan_approach(self, cube_pos_xy):
        push_dir = CUBE_TARGET - cube_pos_xy
        push_dir /= np.linalg.norm(push_dir) + 1e-9
        fn = best_face_normal(push_dir)

        self._push_dir    = push_dir
        self._face_normal = fn
        self._theta_ee    = np.arctan2(push_dir[1], push_dir[0])
        self._f_filt      = np.zeros(2)

        p_hover   = hover_point_xy(cube_pos_xy, fn)
        p_contact = contact_point_xy(cube_pos_xy, fn)
        self._waypoints  = [
            np.array([p_hover[0],   p_hover[1],   APPROACH_Z]),
            np.array([p_hover[0],   p_hover[1],   Z_PUSH    ]),
            np.array([p_contact[0], p_contact[1], Z_PUSH    ]),
        ]
        self._wp_idx     = 0
        self._ctrl_state = STATE_APPROACH
        print(f"\n[Plan] cube={np.round(cube_pos_xy,3)}  push_dir={np.round(push_dir,3)}"
              f"  face={fn}")

    def _plan_reposition(self, cube_pos_xy, p_sph_current):
        """
        Reposition when contact is lost with 4 waypoints
          WP0 : Vertical grinding to get above the cube
          WP1 : LAteral displacement to be next to the cube
          WP2 : Get down to the middle of the cube 
          WP3 : go to contact
        """
        push_dir = CUBE_TARGET - cube_pos_xy
        push_dir /= np.linalg.norm(push_dir) + 1e-9
        fn = best_face_normal(push_dir)

        self._push_dir    = push_dir
        self._face_normal = fn
        self._theta_ee    = np.arctan2(push_dir[1], push_dir[0])
        self._f_filt      = np.zeros(2)

        retrait_xy = cube_pos_xy + (CUBE_HALF + RETRAIT) * fn
        p_hover    = hover_point_xy(cube_pos_xy, fn)
        p_contact  = contact_point_xy(cube_pos_xy, fn)

        self._waypoints  = [
            # WP0 
            np.array([p_sph_current[0], p_sph_current[1], APPROACH_Z]),
            # WP1 
            np.array([retrait_xy[0],    retrait_xy[1],    APPROACH_Z]),
            # WP2 
            np.array([p_hover[0],       p_hover[1],       Z_PUSH    ]),
            # WP3 
            np.array([p_contact[0],     p_contact[1],     Z_PUSH    ]),
        ]
        self._wp_idx     = 0
        self._ctrl_state = STATE_REPOSITION
        print(f"\n[Repos] cube={np.round(cube_pos_xy,3)}  "
              f"push_dir={np.round(push_dir,3)}  face={fn}")
        print(f"  WP0 lift  : {np.round(self._waypoints[0],3)}")
        print(f"  WP1 retrait: {np.round(self._waypoints[1],3)}")
        print(f"  WP2 hover  : {np.round(self._waypoints[2],3)}")
        print(f"  WP3 contact: {np.round(self._waypoints[3],3)}")

    # ── Commande ─────────────────────────────────────────────────────────────

    def _update(self, context, discrete_state):
        x  = self._state_port.Eval(context)
        nq, nv = self._nq, self._nv
        t  = context.get_time()

        self.plant.SetPositions(self.plant_context_ad,  x[:nq])
        self.plant.SetVelocities(self.plant_context_ad, x[nq:])

        _, Jv         = self._get_jacobians()
        p_sph, R_hand = self._get_sphere_center()
        v_arm  = x[nq + self._arm_vel_idx]
        v_sph  = Jv @ v_arm
        g_arm  = self.plant.CalcGravityGeneralizedForces(
                     self.plant_context_ad)[self._arm_vel_idx]
        cube_xy = self._get_cube_xy(x[:nq])

        F_floor = 1000.0 * max(0.0, Z_FLOOR - p_sph[2]) * np.array([0, 0, 1.0])

        # ── DONE ─────────────────────────────────────────────────────────────
        if self._ctrl_state == STATE_DONE:
            tau_total = np.zeros(nv)
            tau_total[self._arm_vel_idx] = Jv.T @ (-Kd_pos * v_sph) - g_arm
            discrete_state.get_mutable_vector().SetFromVector(tau_total)
            return

        # ── APPROACH / REPOSITION ─────────────────────────────────────────
        if self._ctrl_state in (STATE_APPROACH, STATE_REPOSITION):
            wp    = self._waypoints[self._wp_idx].copy()
            wp[2] = max(wp[2], Z_FLOOR)
            err   = np.linalg.norm(p_sph - wp)

            if err < WAYPOINT_THRESHOLD:
                if self._wp_idx < len(self._waypoints) - 1:
                    self._wp_idx += 1
                    print(f"[t={t:.2f}s][{self._ctrl_state}] → WP{self._wp_idx} "
                          f"{np.round(self._waypoints[self._wp_idx],3)}")
                else:
                    self._ctrl_state   = STATE_PUSH
                    self._no_contact_t = 0.0
                    print(f"[t={t:.2f}s] ══ PUSH ══")

            F_pos   = Kp_pos * (wp - p_sph) - Kd_pos * v_sph + F_floor
            tau_arm = Jv.T @ F_pos

        # ── PUSH ─────────────────────────────────────────────────────────────
        else:
            dist = np.linalg.norm(cube_xy - CUBE_TARGET)
            if dist < DONE_THRESHOLD:
                self._ctrl_state = STATE_DONE
                print(f"[t={t:.1f}s] ✓ DONE  dist={dist:.4f}m")
                tau_total = np.zeros(nv)
                tau_total[self._arm_vel_idx] = Jv.T @ (-Kd_pos * v_sph) - g_arm
                discrete_state.get_mutable_vector().SetFromVector(tau_total)
                return

            push_vec = CUBE_TARGET - cube_xy
            push_dir = push_vec / (np.linalg.norm(push_vec) + 1e-9)
            n_d      = np.array([-push_dir[1], push_dir[0]])
            theta_d  = np.arctan2(push_dir[1], push_dir[0])
            self._push_dir = push_dir

            # Estimated force
            f_raw = estimate_contact_force(p_sph[:2], push_dir, cube_xy)
            self._f_filt  = BETA * f_raw + (1 - BETA) * self._f_filt
            f_norm_val    = np.linalg.norm(self._f_filt)
            self._contact = f_norm_val >= F_MIN

            # Filtering
            candidates = np.array([[1,0],[-1,0],[0,1],[0,-1]], float)
            fn_active  = candidates[np.argmax(candidates @ (p_sph[:2] - cube_xy))]
            d_face = float((p_sph[:2] - (cube_xy + CUBE_HALF * fn_active)) @ fn_active)
            if d_face > SPHERE_RADIUS + 0.015:
                self._f_filt *= 0.3

            # Contact lost
            if not self._contact:
                self._no_contact_t += 0.001
                if self._no_contact_t > NO_CONTACT_TIMEOUT:
                    # Verifiy if done before repositioning
                    if dist < DONE_THRESHOLD * 2:   # Security chech
                        self._ctrl_state = STATE_DONE
                        print(f"[t={t:.1f}s] ✓ DONE (contact perdu près cible)  "
                              f"dist={dist:.4f}m")
                        tau_total = np.zeros(nv)
                        tau_total[self._arm_vel_idx] = Jv.T @ (-Kd_pos * v_sph) - g_arm
                        discrete_state.get_mutable_vector().SetFromVector(tau_total)
                        return

                    print(f"[t={t:.2f}s] ⚠ Contact perdu → repositionnement "
                          f"(dist={dist:.3f}m)")
                    # reposition with respect to CURRENT cube pose
                    self._plan_reposition(cube_xy, p_sph)
                    wp    = self._waypoints[0].copy()
                    wp[2] = max(wp[2], Z_FLOOR)
                    tau_total = np.zeros(nv)
                    tau_total[self._arm_vel_idx] = (
                        Jv.T @ (Kp_pos * (wp - p_sph) - Kd_pos * v_sph + F_floor)
                        - g_arm)
                    discrete_state.get_mutable_vector().SetFromVector(tau_total)
                    return
            else:
                self._no_contact_t = 0.0

            # Angle Heins
            if self._contact:
                theta_f = np.arctan2(self._f_filt[1], self._f_filt[0])
                delta_f = (theta_f - theta_d + np.pi) % (2*np.pi) - np.pi
                delta_c = float(n_d @ (p_sph[:2] - cube_xy))
                theta_p = theta_d + (K_F + 1) * delta_f + K_C * delta_c
            else:
                theta_p = theta_d

            self._theta_ee = theta_p

            # Velocity decompisition
            v_push = V_PUSH * np.array([np.cos(theta_p), np.sin(theta_p)])
            lateral_error = float(n_d @ (cube_xy - p_sph[:2]))
            v_lat_xy      = K_LAT * lateral_error * n_d
            vee = v_push + v_lat_xy

            if f_norm_val > F_MAX:
                vee *= F_MAX / f_norm_val

            # Z regulation(not sure)
            vz = np.clip(300.0 * 0.001 * (Z_PUSH - p_sph[2])
                         - 40.0 * 0.001 * v_sph[2], -0.010, 0.010)

            vcmd    = np.array([vee[0], vee[1], vz])
            F_trans = 600.0 * (vcmd - v_sph) + F_floor
            tau_arm = Jv.T @ F_trans

        tau_total = np.zeros(nv)
        tau_total[self._arm_vel_idx] = tau_arm - g_arm
        discrete_state.get_mutable_vector().SetFromVector(tau_total)

    def _log(self, context, mode=None):
        x       = self._state_port.Eval(context)
        q       = x[:self._nq]
        CalcRobotDynamics(self.plant, q=q, v=x[self._nq:])
        p_sph, R = self._get_sphere_center()
        cube_xy  = self._get_cube_xy(q)
        f_n      = np.linalg.norm(self._f_filt)
        rpy      = RollPitchYaw(R).vector()
        dist     = np.linalg.norm(cube_xy - CUBE_TARGET)
        push_dir = self._push_dir
        n_d      = np.array([-push_dir[1], push_dir[0]])
        lat_err  = float(n_d @ (cube_xy - p_sph[:2])) if np.linalg.norm(push_dir) > 0 else 0.0

        print(f"[t={context.get_time():.1f}s][{self._ctrl_state}] "
              f"sph=({p_sph[0]:.3f},{p_sph[1]:.3f},{p_sph[2]:.3f})  "
              f"cube=({cube_xy[0]:.3f},{cube_xy[1]:.3f})  "
              f"dist={dist:.3f}m  lat_err={lat_err*1000:.1f}mm  "
              f"||f||={f_n:.3f}N  {'CONTACT' if self._contact else 'no-contact'}  "
              f"pitch={np.degrees(rpy[1]):.1f}°")


# ═══════════════════════════════════════════════════════════════════════════════
#  VISUALISATION
# ═══════════════════════════════════════════════════════════════════════════════
def plot_results(logger_state, sim_context, best_q7):
    log  = logger_state.FindLog(sim_context)
    time = log.sample_times()
    data = log.data()
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for i in range(7):
        axes[i].plot(time, data[i, :], label='q')
        axes[i].axhline(best_q7[i], ls='--', color='r', label='q_ref')
        axes[i].set_ylabel(f'J{i+1} [rad]')
        axes[i].legend(fontsize=7)
        axes[i].grid(True)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('Joint Positions')
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
#  Simulation and scene
# ═══════════════════════════════════════════════════════════════════════════════
def create_sim_scene(sim_time_step):
    builder   = DiagramBuilder()
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser    = Parser(plant)
    parser.AddModelsFromUrl("file://" + os.path.abspath(ROBOT_URDF_PATH))
    parser.AddModelsFromUrl("file://" + os.path.abspath(SCENE_SDF_FILENAME))

    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("panda_link0",
                         plant.GetModelInstanceByName("panda")),
                     ROBOT_BASE_OFFSET)
    plant.Finalize()
    print("Positions:", plant.num_positions(), " Velocities:", plant.num_velocities())

    best_q7 = [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.0]
    for name, val in zip(["panda_joint1","panda_joint2","panda_joint3",
                           "panda_joint4","panda_joint5","panda_joint6","panda_joint7"],
                         best_q7):
        plant.GetJointByName(name).set_default_angle(val)

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    ctrl = builder.AddNamedSystem("PushController", PushController(plant, best_q7))
    des  = builder.AddNamedSystem("DesiredPos", ConstantVectorSource(best_q7))

    builder.Connect(plant.get_state_output_port(), ctrl.GetInputPort("Current_state"))
    builder.Connect(ctrl.GetOutputPort("tau_u"),   plant.GetInputPort("applied_generalized_force"))
    builder.Connect(des.get_output_port(),          ctrl.GetInputPort("Desired_state"))

    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    logger.set_name("State logger")
    return builder.Build(), logger, best_q7


def run_simulation(sim_time_step=0.001):
    diagram, logger, best_q7 = create_sim_scene(sim_time_step)
    sim = Simulator(diagram)
    sim.Initialize()
    sim.set_target_realtime_rate(1.0)

    pydot.graph_from_dot_data(
        diagram.GetGraphvizString(max_depth=2))[0].write_png("figures/block_diagram.png")

    meshcat.StartRecording()
    sim.AdvanceTo(30.0)
    meshcat.PublishRecording()
    plot_results(logger, sim.get_context(), best_q7)


run_simulation()
