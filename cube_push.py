"""
push_generalized_fixed.py
─────────────────────────────────────────────────────────────────────────────
Version corrigée de push_generalized.py — 4 corrections pour le cylindre.

CORRECTIONS APPLIQUÉES
───────────────────────
  FIX 1 — APPROACH_Z 0.35 → 0.50
    Le bras (links 5-8) descendait à 0.35m et poussait le cylindre pendant
    l'approche latérale avant même d'entrer en PUSH. À 0.50m le bras reste
    au-dessus du cylindre (h=0.10m) pendant tout le déplacement latéral.

  FIX 2 — _extract_drake_contact cherche panda_link8 en plus de panda_hand
    Contact réel = panda_link8 (sphère à xyz="0.042 0.042 -0.02" dans URDF),
    pas panda_hand. Drake ne trouvait rien → src=est permanent, n_real jamais
    initialisé, PI avec n_use wrong direction → cylindre bloqué.

  FIX 3 — face_normal mis à jour à chaque pas pour le cylindre
    Pour un cylindre la normale est radiale = -push_dir courant. Le code la
    fixait une fois à la planification. Après que le cylindre a bougé,
    face_normal=[0.486,0.874] avec push_dir≈[-0.987,0.159] → estimateur 180°
    faux → v_PI dans le mauvais sens → aucun mouvement à partir de t=7s.

  FIX 4 — détection de contact radiale pour le cylindre
    L'ancienne détection utilisait (p_sph - face_center) @ face_normal,
    conçu pour une face plane. Pour un cylindre, la distance radiale depuis
    le centre est géométriquement exacte.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tutorial_scripts"))
import numpy as np
import matplotlib.pyplot as plt
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, ContactResults
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.primitives import ConstantVectorSource, LogVectorOutput
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.visualization import AddDefaultVisualization
from helper.dynamics import CalcRobotDynamics

ROBOT_URDF_PATH = os.path.join("..", "models", "descriptions", "robots", "arms",
                                "franka_description", "urdf", "panda_arm_hand_sphere.urdf")

# ═══════════════════════════════════════════════════════════════════════════════
#  GÉOMÉTRIE
# ═══════════════════════════════════════════════════════════════════════════════
FINGER_TIP_OFFSET = np.array([0.0, 0.0, 0.0])
SPHERE_RADIUS     = 0.05
TABLE_TOP_Z       = 0.050
Z_PUSH            = TABLE_TOP_Z + SPHERE_RADIUS   # 0.10 m
#Z_PUSH            = 0.085  # 0.10 m
Z_FLOOR           = Z_PUSH - 0.003
ROBOT_BASE_OFFSET = RigidTransform(RotationMatrix.Identity(), [-0.10, 0.0, 0.0])

DEFAULT_OBJECT_POS    = np.array([0.35, 0.05, 0.1])
DEFAULT_OBJECT_TARGET = np.array([0.80, 0.30])

# ═══════════════════════════════════════════════════════════════════════════════
#  PARAMÈTRES CONTRÔLEUR
# ═══════════════════════════════════════════════════════════════════════════════
V_PUSH = 0.06
F_MIN  = 0.05
F_MAX  = 15.0
BETA   = 0.30

K_F   = 0.15
K_C   = 0.10
K_LAT = 1.5

Kp_pos = 400.0
Kd_pos = 80.0

# FIX 1 : APPROACH_Z relevé de 0.35 → 0.50
# Le bras (liens 5-8) avait des sphères de collision qui touchaient le
# cylindre pendant le déplacement latéral à Z=0.35m.
APPROACH_Z         = 0.50
WAYPOINT_THRESHOLD = 0.020

DONE_THRESHOLD     = 0.01
NO_CONTACT_TIMEOUT = 5.0
RETRAIT            = 0.10
DONE_RETRACT_TIME  = 2.0
DONE_RETRACT_SPEED = 0.04

K_NORMAL  = 700.0
K_TANGENT = 50.0
MU_PUSHER = 0.3

FN_DESIRED      = 10.0
KP_FN           = 0.05
KI_FN           = 0.01
V_NORMAL_MAX    = 0.015
FN_WINDUP_LIMIT = 0.3

CONTACT_DIST_MARGIN = 0.005
DELTA_C_FILTER      = 0.05


# ═══════════════════════════════════════════════════════════════════════════════
#  FONCTIONS GÉOMÉTRIQUES GÉNÉRALISÉES
# ═══════════════════════════════════════════════════════════════════════════════

def best_face_normal(push_dir_xy):
    candidates = np.array([[1,0],[-1,0],[0,1],[0,-1]], float)
    return candidates[np.argmin(candidates @ push_dir_xy)]


def get_approach_direction(push_dir_xy, shape_type):
    """
    Direction approche = direction depuis centre objet vers point de contact.
    Cube : snap sur l'axe de face le plus proche (-push_dir en discret).
    Cylindre : -push_dir continu (distance centre→surface = R dans ts directions).
    """
    if shape_type == "cube":
        return best_face_normal(push_dir_xy)
    elif shape_type == "cylinder":
        return -push_dir_xy / (np.linalg.norm(push_dir_xy) + 1e-9)
    else:
        raise ValueError(f"shape_type inconnu: {shape_type}")


def get_c_squared(shape_type, half_extent):
    """
    Rayon de giration² analytique (pression uniforme) :
      cube     : c² = 2a²/3
      cylindre : c² = R²/2
    """
    if shape_type == "cube":
        return 2.0 * half_extent**2 / 3.0
    elif shape_type == "cylinder":
        return half_extent**2 / 2.0
    else:
        raise ValueError(f"shape_type inconnu: {shape_type}")


def contact_point_xy(object_pos_xy, fn, half_extent):
    return object_pos_xy + (half_extent + SPHERE_RADIUS - 0.005) * fn


def hover_point_xy(object_pos_xy, fn, half_extent):
    return object_pos_xy + (half_extent + SPHERE_RADIUS + 0.12) * fn


def estimate_contact_wrench(p_sph_xy, v_sph_xy, object_pos_xy, face_normal, half_extent):
    """Estimateur géométrique fn/ft — fallback si Drake indisponible."""
    tangent     = np.array([-face_normal[1], face_normal[0]])
    face_center = object_pos_xy + half_extent * face_normal
    d_normal    = float((p_sph_xy - face_center) @ face_normal)
    pen         = float(np.clip(SPHERE_RADIUS - d_normal, 0.0, 0.02))
    if pen < 1e-5:
        return np.zeros(2), 0.0, 0.0
    fn_scalar = K_NORMAL * pen
    f_normal  = fn_scalar * face_normal
    v_lat     = float(v_sph_xy @ tangent)
    ft_scalar = float(np.clip(-K_TANGENT * v_lat,
                               -MU_PUSHER * fn_scalar, MU_PUSHER * fn_scalar))
    return f_normal + ft_scalar * tangent, fn_scalar, ft_scalar


# ═══════════════════════════════════════════════════════════════════════════════
#  RÉGULATEUR PI SUR fn
# ═══════════════════════════════════════════════════════════════════════════════

class ContactForceRegulator:
    def __init__(self):
        self._integral = 0.0
        self._last_t   = -1.0

    def reset(self):
        self._integral = 0.0
        self._last_t   = -1.0

    def compute(self, t, fn_measured, n_real, push_dir_corrected, in_contact):
        dt = (t - self._last_t) if self._last_t >= 0 else 0.001
        self._last_t = t
        v_push = V_PUSH * push_dir_corrected
        e_fn   = FN_DESIRED - fn_measured
        if not in_contact:
            self._integral = 0.0
            v_approach = min(V_NORMAL_MAX, KP_FN * FN_DESIRED) * (-n_real)
            return v_push + v_approach, 0.0, e_fn
        self._integral = np.clip(self._integral + e_fn * dt,
                                 -FN_WINDUP_LIMIT, FN_WINDUP_LIMIT)
        v_pi = np.clip(KP_FN * e_fn + KI_FN * self._integral,
                       -V_NORMAL_MAX, V_NORMAL_MAX)
        return v_push + v_pi * (-n_real), v_pi, e_fn


# ═══════════════════════════════════════════════════════════════════════════════
#  ÉTATS FSM
# ═══════════════════════════════════════════════════════════════════════════════
STATE_APPROACH   = "APPROACH"
STATE_PUSH       = "PUSH"
STATE_REPOSITION = "REPOSITION"
STATE_DONE       = "DONE"


# ═══════════════════════════════════════════════════════════════════════════════
#  CONTRÔLEUR
# ═══════════════════════════════════════════════════════════════════════════════
class PushController(LeafSystem):

    def __init__(self, plant, init_q7, shape_type="cube", half_extent=0.05,
                 object_model_name="object", object_pos=None, object_target=None):
        super().__init__()
        nq = plant.num_positions()
        nv = plant.num_velocities()

        self._state_port   = self.DeclareVectorInputPort("Current_state", size=nq + nv)
        self._desired_port = self.DeclareVectorInputPort("Desired_state",  size=7)
        self._contact_results_port = self.DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults()))

        self.plant            = plant
        self.plant_context_ad = plant.CreateDefaultContext()
        self._nq, self._nv    = nq, nv

        self.shape_type    = shape_type
        self.half_extent   = half_extent
        self.c_sq          = get_c_squared(shape_type, half_extent)
        self.object_pos    = np.array(object_pos) if object_pos is not None else DEFAULT_OBJECT_POS.copy()
        self.object_target = np.array(object_target) if object_target is not None else DEFAULT_OBJECT_TARGET.copy()

        self._panda_model = plant.GetModelInstanceByName("panda")
        vel_idx = []
        for ji in plant.GetJointIndices(self._panda_model):
            jt = plant.get_joint(ji)
            if jt.num_velocities() > 0:
                vel_idx.extend(range(jt.velocity_start(),
                                     jt.velocity_start() + jt.num_velocities()))
        self._arm_vel_idx = np.array(vel_idx, dtype=int)

        self._object_model = plant.GetModelInstanceByName(object_model_name)
        obj_idx = []
        for ji in plant.GetJointIndices(self._object_model):
            jt = plant.get_joint(ji)
            if jt.num_positions() > 0:
                obj_idx.extend(range(jt.position_start(),
                                     jt.position_start() + jt.num_positions()))
        self._object_pos_idx = np.array(obj_idx, dtype=int)

        self._hand_body   = plant.GetBodyByName("panda_hand")
        self._object_body = self._find_first_body(plant, object_model_name)

        # FIX 2 : construire l'ensemble des corps "main" (panda_hand + panda_link8)
        self._hand_indices = {self._hand_body.index()}
        try:
            self._hand_indices.add(plant.GetBodyByName("panda_link8").index())
        except Exception:
            pass

        self._ctrl_state   = STATE_APPROACH
        self._waypoints    = []
        self._wp_idx       = 0
        self._face_normal  = np.zeros(2)
        self._face_tangent = np.zeros(2)
        self._push_dir     = np.zeros(2)
        self._fn_filt      = 0.0
        self._ft_filt      = 0.0
        self._delta_c_filt = 0.0
        self._contact      = False
        self._no_contact_t = 0.0
        self._prev_dist    = None
        self._last_dist    = None

        self._force_reg   = ContactForceRegulator()
        self._n_real_filt = np.zeros(2)
        self._n_real_set  = False
        self._force_log   = []
        self._done_time     = None
        self._done_sim_time = None

        plant.SetPositions(self.plant_context_ad, self._panda_model, np.array(init_q7))
        p0, _ = self._get_sphere_center()
        print(f"[Controller] shape={shape_type} c²={self.c_sq:.4f} "
              f"sphere_init={np.round(p0,4)}")
        self._plan_approach(self.object_pos[:2])

        state_idx = self.DeclareDiscreteState(nv)
        self.DeclareStateOutputPort("tau_u", state_idx)
        self.DeclarePeriodicDiscreteUpdateEvent(1/1000, 0.0, self._update)
        self.DeclarePeriodicPublishEvent(1, 0, self._log)

    def _find_first_body(self, plant, model_name):
        model = plant.GetModelInstanceByName(model_name)
        for bi in plant.GetBodyIndices(model):
            body = plant.get_body(bi)
            if body.name() != "world":
                return body
        return None

    # ── Cinématique ──────────────────────────────────────────────────────────

    def _get_sphere_center(self):
        ee   = self.plant.GetFrameByName("panda_hand")
        X_WE = self.plant.CalcRelativeTransform(
            self.plant_context_ad, self.plant.world_frame(), ee)
        R = X_WE.rotation()
        return X_WE.translation() + R.matrix() @ FINGER_TIP_OFFSET, R

    def _get_jacobians(self):
        ee     = self.plant.GetFrameByName("panda_hand")
        J_full = self.plant.CalcJacobianSpatialVelocity(
            self.plant_context_ad, JacobianWrtVariable.kV,
            ee, FINGER_TIP_OFFSET,
            self.plant.world_frame(), self.plant.world_frame())
        J = J_full[:, self._arm_vel_idx]
        return J[:3, :], J[3:, :]

    def _get_object_xy(self, q):
        if len(self._object_pos_idx) == 7:
            return q[self._object_pos_idx[4:7]][:2].copy()
        return self.object_pos[:2].copy()

    # ── FIX 2 : extraction forces Drake — panda_hand ET panda_link8 ──────────

    def _extract_drake_contact(self, context):
        """
        Cherche le contact entre l'objet et n'importe quel corps de la
        zone main (panda_hand + panda_link8). L'URDF montre que panda_link8
        a une sphère de collision à xyz="0.042 0.042 -0.02" qui peut être
        le premier corps à toucher l'objet dans certaines configurations.
        """
        cr         = self._contact_results_port.Eval(context)
        n_contacts = cr.num_point_pair_contacts()
        if n_contacts == 0 or self._object_body is None:
            return None, 0.0, 0.0

        obj_idx   = self._object_body.index()
        f_on_hand = np.zeros(3)
        found     = False

        for i in range(n_contacts):
            info   = cr.point_pair_contact_info(i)
            bA, bB = info.bodyA_index(), info.bodyB_index()
            bodyA = self.plant.get_body(bA)
            bodyB = self.plant.get_body(bB)
            if bA in self._hand_indices and bB == obj_idx:
                f_on_hand += -np.array(info.contact_force())
                found = True
            elif bB in self._hand_indices and bA == obj_idx:
                f_on_hand += np.array(info.contact_force())
                found = True
        if not found:
            return None, 0.0, 0.0

        f_xy  = f_on_hand[:2]
        f_mag = np.linalg.norm(f_xy)
        if f_mag < 1e-4:
            return None, 0.0, 0.0

        n_real   = f_xy / f_mag
        t_real   = np.array([-n_real[1], n_real[0]])
        fn_drake = float(f_xy @ n_real)
        ft_drake = float(f_xy @ t_real)
        return n_real, fn_drake, ft_drake

    # ── Planification ────────────────────────────────────────────────────────

    def _reset_push_state(self):
        self._fn_filt = 0.0; self._ft_filt = 0.0; self._delta_c_filt = 0.0
        self._force_reg.reset()
        self._n_real_set = False; self._n_real_filt = np.zeros(2)
        self._prev_dist  = None

    def _plan_approach(self, object_pos_xy):
        push_dir = self.object_target - object_pos_xy
        push_dir /= np.linalg.norm(push_dir) + 1e-9
        fn = get_approach_direction(push_dir, self.shape_type)
        self._push_dir     = push_dir
        self._face_normal  = fn
        self._face_tangent = np.array([-fn[1], fn[0]])
        self._reset_push_state()
        p_hover   = hover_point_xy(object_pos_xy, fn, self.half_extent)
        p_contact = contact_point_xy(object_pos_xy, fn, self.half_extent)
        self._waypoints = [
            np.array([p_hover[0],   p_hover[1],   APPROACH_Z]),
            np.array([p_hover[0],   p_hover[1],   Z_PUSH    ]),
            np.array([p_contact[0], p_contact[1], Z_PUSH    ]),
        ]
        self._wp_idx = 0; self._ctrl_state = STATE_APPROACH
        print(f"\n[Plan] obj={np.round(object_pos_xy,3)} "
              f"push_dir={np.round(push_dir,3)} "
              f"approach_dir={np.round(fn,3)} ({self.shape_type})")

    def _plan_reposition(self, object_pos_xy, p_sph_current):
        push_dir = self.object_target - object_pos_xy
        push_dir /= np.linalg.norm(push_dir) + 1e-9
        fn = get_approach_direction(push_dir, self.shape_type)
        self._push_dir     = push_dir
        self._face_normal  = fn
        self._face_tangent = np.array([-fn[1], fn[0]])
        self._reset_push_state()
        retrait_xy = object_pos_xy + (self.half_extent + RETRAIT) * fn
        p_hover    = hover_point_xy(object_pos_xy, fn, self.half_extent)
        p_contact  = contact_point_xy(object_pos_xy, fn, self.half_extent)
        self._waypoints = [
            np.array([p_sph_current[0], p_sph_current[1], APPROACH_Z]),
            np.array([retrait_xy[0],    retrait_xy[1],    APPROACH_Z]),
            np.array([p_hover[0],       p_hover[1],       Z_PUSH    ]),
            np.array([p_contact[0],     p_contact[1],     Z_PUSH    ]),
        ]
        self._wp_idx = 0; self._ctrl_state = STATE_REPOSITION
        print(f"\n[Repos] obj={np.round(object_pos_xy,3)} approach={np.round(fn,3)}")

    # ── Boucle de contrôle à 1 kHz ───────────────────────────────────────────

    def _update(self, context, discrete_state):
        x      = self._state_port.Eval(context)
        nq, nv = self._nq, self._nv
        t      = context.get_time()

        self.plant.SetPositions(self.plant_context_ad,  x[:nq])
        self.plant.SetVelocities(self.plant_context_ad, x[nq:])

        _, Jv    = self._get_jacobians()
        p_sph, _ = self._get_sphere_center()
        v_arm    = x[nq + self._arm_vel_idx]
        v_sph    = Jv @ v_arm
        g_arm    = self.plant.CalcGravityGeneralizedForces(
                       self.plant_context_ad)[self._arm_vel_idx]
        object_xy = self._get_object_xy(x[:nq])
        F_floor   = 1000.0 * max(0.0, Z_FLOOR - p_sph[2]) * np.array([0, 0, 1.0])
        tau_total = np.zeros(nv)

        # ── DONE ─────────────────────────────────────────────────────────────
        if self._ctrl_state == STATE_DONE:
            if self._done_time is None:
                self._done_time = t
            t_since = t - self._done_time
            if t_since < DONE_RETRACT_TIME:
                n_ret    = self._n_real_filt if self._n_real_set else self._face_normal
                v_ret_2d = DONE_RETRACT_SPEED * n_ret
                vz       = np.clip(300*0.001*(Z_PUSH-p_sph[2]) - 40*0.001*v_sph[2],
                                   -0.010, 0.010)
                F_trans  = 600.0 * (np.array([v_ret_2d[0], v_ret_2d[1], vz]) - v_sph)
                tau_total[self._arm_vel_idx] = Jv.T @ F_trans - g_arm
            else:
                tau_total[self._arm_vel_idx] = Jv.T @ (-Kd_pos * v_sph) - g_arm
            discrete_state.get_mutable_vector().SetFromVector(tau_total)
            return

        # ── APPROACH / REPOSITION ─────────────────────────────────────────────
        if self._ctrl_state in (STATE_APPROACH, STATE_REPOSITION):
            wp    = self._waypoints[self._wp_idx].copy()
            wp[2] = max(wp[2], Z_FLOOR)
            if np.linalg.norm(p_sph - wp) < WAYPOINT_THRESHOLD:
                if self._wp_idx < len(self._waypoints) - 1:
                    self._wp_idx += 1
                    print(f"[t={t:.2f}s][{self._ctrl_state}] → WP{self._wp_idx} "
                          f"{np.round(self._waypoints[self._wp_idx],3)}")
                else:
                    self._ctrl_state   = STATE_PUSH
                    self._no_contact_t = 0.0
                    self._prev_dist    = None
                    print(f"[t={t:.2f}s] ══ PUSH ══")
            F_pos   = Kp_pos * (wp - p_sph) - Kd_pos * v_sph + F_floor
            tau_total[self._arm_vel_idx] = Jv.T @ F_pos - g_arm
            discrete_state.get_mutable_vector().SetFromVector(tau_total)
            return

        # ── PUSH ─────────────────────────────────────────────────────────────
        dist = np.linalg.norm(object_xy - self.object_target)
        self._last_dist = dist

        if dist < DONE_THRESHOLD:
            self._ctrl_state    = STATE_DONE
            self._done_time     = None
            self._done_sim_time = t
            print(f"[t={t:.1f}s] ✓ DONE  dist={dist*1000:.1f}mm  shape={self.shape_type}")
            tau_total[self._arm_vel_idx] = Jv.T @ (-Kd_pos * v_sph) - g_arm
            discrete_state.get_mutable_vector().SetFromVector(tau_total)
            return

        push_vec = self.object_target - object_xy
        push_dir = push_vec / (np.linalg.norm(push_vec) + 1e-9)
        self._push_dir = push_dir

        # FIX 3 : mise à jour continue de face_normal pour le cylindre.
        # Pour un cylindre, la normale de contact est toujours radiale
        # depuis le centre = -push_dir courant. Sans cette mise à jour,
        # après que le cylindre a bougé, l'estimateur utilise une normale
        # figée depuis la planification initiale → erreur 180° possible.
        if self.shape_type == "cylinder" and not self._n_real_set:
            self._face_normal  = -push_dir.copy()
            self._face_tangent = np.array([-self._face_normal[1],
                                            self._face_normal[0]])

        # ── FIX 2 : forces Drake (panda_hand + panda_link8) ──────────────────
        n_drake, fn_drake, ft_drake = self._extract_drake_contact(context)
        drake_ok = (n_drake is not None and fn_drake > F_MIN)

        if drake_ok:
            if not self._n_real_set:
                self._n_real_filt = n_drake.copy(); self._n_real_set = True
            else:
                nb = 0.7 * n_drake + 0.3 * self._n_real_filt
                self._n_real_filt = nb / (np.linalg.norm(nb) + 1e-9)
        else:
            if not self._n_real_set:
                self._n_real_filt = self._face_normal.copy()
        n_use = self._n_real_filt

        # Estimateur géométrique (fallback)
        _, fn_est, ft_est = estimate_contact_wrench(
            p_sph[:2], v_sph[:2], object_xy, self._face_normal, self.half_extent)

        fn_use = fn_drake if drake_ok else fn_est
        ft_use = ft_drake if drake_ok else ft_est
        self._fn_filt = BETA * fn_use + (1 - BETA) * self._fn_filt
        self._ft_filt = BETA * ft_use + (1 - BETA) * self._ft_filt

        if int(t * 1000) % 10 == 0:
            self._force_log.append({
                "t": t,
                "fn_drake": fn_drake if drake_ok else 0.0,
                "ft_drake": ft_drake if drake_ok else 0.0,
                "fn_est": fn_est, "ft_est": ft_est, "drake_ok": drake_ok,
            })

        # FIX 4 : détection de contact adaptée à la forme.
        # Cube : face plane → distance normale à la face.
        # Cylindre : distance radiale depuis le centre (géométriquement exacte).
        if self.shape_type == "cylinder":
            dist_radial     = np.linalg.norm(p_sph[:2] - object_xy)
            in_contact_geom = dist_radial < (self.half_extent + SPHERE_RADIUS
                                             + CONTACT_DIST_MARGIN)
        else:
            face_center     = object_xy + self.half_extent * self._face_normal
            d_contact       = float((p_sph[:2] - face_center) @ self._face_normal)
            in_contact_geom = d_contact < (SPHERE_RADIUS + CONTACT_DIST_MARGIN)

        in_contact_est  = self._fn_filt >= F_MIN
        self._contact   = in_contact_geom or in_contact_est or drake_ok

        # Feedback py depuis ft/fn
        py_from_force = self.c_sq * self._ft_filt / (self._fn_filt + 1e-9)
        delta_f       = np.arctan2(self._ft_filt, self._fn_filt + 1e-9)

        theta_desired = np.arctan2(push_dir[1], push_dir[0])
        if self._contact and self._fn_filt >= F_MIN:
            theta_push = theta_desired + K_F * delta_f + K_C * py_from_force
        else:
            theta_push = theta_desired
        push_dir_corrected = np.array([np.cos(theta_push), np.sin(theta_push)])

        # Timeout sans contact
        if not self._contact:
            self._no_contact_t += 0.001
            if self._prev_dist is not None and (self._prev_dist - dist) > 0.0005:
                self._no_contact_t = max(0.0, self._no_contact_t - 0.002)
            if self._no_contact_t > NO_CONTACT_TIMEOUT:
                if dist < DONE_THRESHOLD * 2:
                    self._ctrl_state = STATE_DONE; self._done_time = None
                    self._done_sim_time = t
                    tau_total[self._arm_vel_idx] = Jv.T @ (-Kd_pos * v_sph) - g_arm
                    discrete_state.get_mutable_vector().SetFromVector(tau_total)
                    return
                print(f"[t={t:.2f}s] ⚠ repos dist={dist*1000:.1f}mm")
                self._plan_reposition(object_xy, p_sph)
                wp = self._waypoints[0].copy(); wp[2] = max(wp[2], Z_FLOOR)
                tau_total[self._arm_vel_idx] = (
                    Jv.T @ (Kp_pos*(wp-p_sph) - Kd_pos*v_sph + F_floor) - g_arm)
                discrete_state.get_mutable_vector().SetFromVector(tau_total)
                return
        else:
            self._no_contact_t = 0.0
        self._prev_dist = dist

        # v_cmd = V_PUSH * push_dir + PI(fn) * (-n_real)
        v_cmd_2d, v_pi, e_fn = self._force_reg.compute(
            t, self._fn_filt, n_use, push_dir_corrected, self._contact)

        # Correction latérale pondérée par la distance
        lat_scale = float(np.clip(dist / 0.15, 0.2, 1.0))
        n_d       = np.array([-push_dir[1], push_dir[0]])
        lat_err   = float(n_d @ (object_xy - p_sph[:2]))
        v_cmd_2d  = v_cmd_2d + K_LAT * lat_scale * lat_err * n_d

        if self._contact and self._fn_filt > 1.0:
            py_measured  = self.c_sq * self._ft_filt / (self._fn_filt + 1e-9)
            # La correction pousse la sphère dans -tangent si py > 0
            # t_face = n_d (déjà calculé ci-dessus — perpendiculaire à push_dir)
            K_PY         = 2.0          # gain à tuner (commence bas)
            v_py_correct = -K_PY * py_measured * n_d
            v_cmd_2d     = v_cmd_2d + v_py_correct

        # Saturation force
        f_norm_total = np.sqrt(self._fn_filt**2 + self._ft_filt**2)
        if f_norm_total > F_MAX:
            v_cmd_2d *= F_MAX / f_norm_total

        # Régulation Z
        vz = np.clip(300*0.001*(Z_PUSH-p_sph[2]) - 40*0.001*v_sph[2], -0.010, 0.010)

        v_cmd_3d = np.array([v_cmd_2d[0], v_cmd_2d[1], vz])
        F_trans  = 600.0 * (v_cmd_3d - v_sph) + F_floor
        tau_arm  = Jv.T @ F_trans
        tau_total[self._arm_vel_idx] = tau_arm - g_arm
        discrete_state.get_mutable_vector().SetFromVector(tau_total)

    # ── Log ──────────────────────────────────────────────────────────────────

    def _log(self, context, mode=None):
        x        = self._state_port.Eval(context)
        q        = x[:self._nq]
        t        = context.get_time()
        CalcRobotDynamics(self.plant, q=q, v=x[self._nq:])
        object_xy = self._get_object_xy(q)
        dist      = np.linalg.norm(object_xy - self.object_target)
        py_mm     = self.c_sq * self._ft_filt / (self._fn_filt + 1e-9) * 1000
        df_deg    = np.degrees(np.arctan2(self._ft_filt, self._fn_filt + 1e-9))
        src       = "Drake" if (self._n_real_set and self._fn_filt > F_MIN) else "est"
        print(f"[t={t:.1f}s][{self._ctrl_state}][{self.shape_type}] "
              f"obj=({object_xy[0]:.3f},{object_xy[1]:.3f}) dist={dist*1000:.1f}mm "
              f"fn={self._fn_filt:.2f}N ft={self._ft_filt:.2f}N "
              f"py={py_mm:.1f}mm δf={df_deg:.1f}° src={src} "
              f"{'CONTACT✓' if self._contact else 'no-contact'}")

    def get_metrics(self):
        log = self._force_log
        if log:
            fn_drake = np.array([e["fn_drake"] for e in log])
            ft_drake = np.array([e["ft_drake"] for e in log])
            fn_est   = np.array([e["fn_est"]   for e in log])
            ft_est   = np.array([e["ft_est"]   for e in log])
            mask     = np.array([e["drake_ok"] for e in log])
            if mask.any():
                fn_rmse   = float(np.sqrt(np.mean((fn_est[mask]-fn_drake[mask])**2)))
                ft_rmse   = float(np.sqrt(np.mean((ft_est[mask]-ft_drake[mask])**2)))
                drake_pct = float(100 * mask.sum() / len(log))
            else:
                fn_rmse = ft_rmse = float("nan"); drake_pct = 0.0
        else:
            fn_rmse = ft_rmse = float("nan"); drake_pct = 0.0
        return {
            "shape": self.shape_type, "half_extent": self.half_extent,
            "success": self._ctrl_state == STATE_DONE,
            "final_dist_mm": float(self._last_dist*1000) if self._last_dist is not None else float("nan"),
            "completion_time_s": self._done_sim_time,
            "fn_rmse": fn_rmse, "ft_rmse": ft_rmse,
            "drake_contact_pct": drake_pct,
        }

    def save_force_validation_plot(self):
        if not self._force_log:
            return
        log      = self._force_log
        t_arr    = np.array([e["t"]        for e in log])
        fn_drake = np.array([e["fn_drake"] for e in log])
        ft_drake = np.array([e["ft_drake"] for e in log])
        fn_est   = np.array([e["fn_est"]   for e in log])
        ft_est   = np.array([e["ft_est"]   for e in log])
        mask     = np.array([e["drake_ok"] for e in log])
        err_fn   = np.where(mask, np.abs(fn_est - fn_drake), np.nan)
        err_ft   = np.where(mask, np.abs(ft_est - ft_drake), np.nan)
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].plot(t_arr, fn_drake, label="fn réel (Drake)", color="steelblue", lw=1.5)
        axes[0].plot(t_arr, fn_est,   label="fn estimé",       color="tomato",    lw=1, ls="--")
        axes[0].axhline(FN_DESIRED, color="green", lw=1, ls=":", label=f"FN*={FN_DESIRED}N")
        axes[0].set_ylabel("fn [N]"); axes[0].legend(fontsize=9); axes[0].grid(True)
        axes[0].set_title(f"Validation force — {self.shape_type}")
        axes[1].plot(t_arr, ft_drake, label="ft réel", color="steelblue", lw=1.5)
        axes[1].plot(t_arr, ft_est,   label="ft estimé", color="tomato", lw=1, ls="--")
        axes[1].axhline(0, color="gray", lw=0.8, ls=":"); axes[1].set_ylabel("ft [N]")
        axes[1].legend(fontsize=9); axes[1].grid(True)
        axes[2].plot(t_arr, err_fn, label="|err fn|", color="darkorange", lw=1.2)
        axes[2].plot(t_arr, err_ft, label="|err ft|", color="purple",     lw=1.2)
        axes[2].set_ylabel("erreur [N]"); axes[2].set_xlabel("temps [s]")
        axes[2].legend(fontsize=9); axes[2].grid(True)
        mean_fn = float(np.nanmean(err_fn)) if mask.any() else 0.0
        mean_ft = float(np.nanmean(err_ft)) if mask.any() else 0.0
        axes[2].set_title(f"Erreur moy fn={mean_fn:.3f}N  ft={mean_ft:.3f}N  "
                          f"Drake={100*mask.sum()/max(1,len(log)):.1f}%")
        plt.tight_layout()
        os.makedirs("figures", exist_ok=True)
        fname = f"figures/force_validation_{self.shape_type}.png"
        plt.savefig(fname, dpi=150); plt.close()
        print(f"[Option 1] → {fname}")
        print(f"[Option 1] fn={mean_fn:.4f}N  ft={mean_ft:.4f}N  "
              f"Drake={100*mask.sum()/max(1,len(log)):.1f}%")


# ═══════════════════════════════════════════════════════════════════════════════
#  SCÈNE ET SIMULATION
# ═══════════════════════════════════════════════════════════════════════════════

def create_sim_scene(sdf_path, shape_type, half_extent, object_model_name="object",
                     object_pos=None, object_target=None, sim_time_step=0.001,
                     render=False, meshcat=None):
    builder   = DiagramBuilder()
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser    = Parser(plant)
    parser.AddModelsFromUrl("file://" + os.path.abspath(ROBOT_URDF_PATH))
    parser.AddModelsFromUrl("file://" + os.path.abspath(sdf_path))
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("panda_link0", plant.GetModelInstanceByName("panda")),
        ROBOT_BASE_OFFSET)
    plant.Finalize()
    print("Positions:", plant.num_positions(), " Velocities:", plant.num_velocities())

    best_q7 = [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.0]
    for name, val in zip(
            ["panda_joint1","panda_joint2","panda_joint3","panda_joint4",
             "panda_joint5","panda_joint6","panda_joint7"], best_q7):
        plant.GetJointByName(name).set_default_angle(val)

    if render and meshcat is not None:
        AddDefaultVisualization(builder=builder, meshcat=meshcat)

    ctrl = builder.AddNamedSystem("PushController", PushController(
        plant, best_q7, shape_type=shape_type, half_extent=half_extent,
        object_model_name=object_model_name, object_pos=object_pos,
        object_target=object_target))
    des  = builder.AddNamedSystem("DesiredPos", ConstantVectorSource(best_q7))

    builder.Connect(plant.get_state_output_port(), ctrl.GetInputPort("Current_state"))
    builder.Connect(ctrl.GetOutputPort("tau_u"),
                    plant.GetInputPort("applied_generalized_force"))
    builder.Connect(des.get_output_port(), ctrl.GetInputPort("Desired_state"))
    builder.Connect(plant.get_contact_results_output_port(),
                    ctrl.GetInputPort("contact_results"))

    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    logger.set_name("State logger")
    return builder.Build(), logger, best_q7, ctrl


def run_trial(sdf_path, shape_type, half_extent, object_model_name="object",
              object_pos=None, object_target=None, sim_time=30.0, render=False):
    meshcat = None
    if render:
        meshcat = StartMeshcat()
        meshcat.Delete()
        meshcat.DeleteAddedControls()

    diagram, logger, best_q7, ctrl = create_sim_scene(
        sdf_path, shape_type, half_extent, object_model_name,
        object_pos, object_target, render=render, meshcat=meshcat)

    sim = Simulator(diagram)
    sim.Initialize()
    if render:
        sim.set_target_realtime_rate(1.0)

    try:
        if render:
            meshcat.StartRecording()
        sim.AdvanceTo(sim_time)
        if render:
            meshcat.PublishRecording()
    except Exception as e:
        print(f"[run_trial] échec: {e}")
        return {"shape": shape_type, "success": False, "error": str(e)}

    ctrl.save_force_validation_plot()
    return ctrl.get_metrics()


# ═══════════════════════════════════════════════════════════════════════════════
#  EXÉCUTION DIRECTE
# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    SHAPE    = "cube"
    SDF_PATH = "models_generated/envcube_test.sdf"
    HALF_EXT = 0.05

    metrics = run_trial(
        sdf_path=SDF_PATH, shape_type=SHAPE, half_extent=HALF_EXT,
        object_model_name="object", sim_time=20.0, render=True,
    )
    print("\n=== Résultat ===")
    for k, v in metrics.items():
        print(f"  {k}: {v}")
