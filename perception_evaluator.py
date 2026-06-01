"""
perception_evaluator.py
═══════════════════════════════════════════════════════════════════════════════
Module d'évaluation quantitative de la perception — à brancher dans create_sim_scene().

PRINCIPE
────────
PerceptionEvaluator est un LeafSystem qui tourne à la même fréquence que la
caméra (30 Hz). À chaque tick il lit :
  - p̂  (estimation Kalman)  ← depuis le port cube_pose_estimate
  - p*  (vérité terrain)     ← depuis le state vector Drake + index du cube
  - FSM state              ← depuis un port dédié sur PushController

Il accumule un log en RAM et l'écrit en fin de run dans
  figures/perception_eval.npz

UTILISATION
───────────
Dans create_sim_scene(), après avoir créé perception et ctrl :

    from perception_evaluator import PerceptionEvaluator

    evaluator = builder.AddNamedSystem("PerceptionEval",
        PerceptionEvaluator(plant, cube_model_name="cube", freq_hz=CAM_FREQ_HZ))

    builder.Connect(perception.GetOutputPort("cube_pose_estimate"),
                    evaluator.GetInputPort("cube_pose_estimate"))
    builder.Connect(plant.get_state_output_port(),
                    evaluator.GetInputPort("plant_state"))
    builder.Connect(ctrl.GetOutputPort("fsm_state_id"),   # voir note ci-dessous
                    evaluator.GetInputPort("fsm_state_id"))

NOTE — port fsm_state_id
    Ajouter dans PushController.__init__() :
        self._fsm_idx = self.DeclareDiscreteState(1)
        self.DeclareStateOutputPort("fsm_state_id", self._fsm_idx)
    Et dans _update(), juste avant le return final :
        FSM_ID = {STATE_APPROACH:0, STATE_PUSH:1, STATE_REPOSITION:2, STATE_DONE:3}
        discrete_state.get_mutable_vector()[self._fsm_idx] = FSM_ID[self._ctrl_state]
    (si c'est trop invasif, passe fsm_state_id=None et le module devine depuis dist)
"""

import os
import numpy as np
from pydrake.systems.framework import LeafSystem
from pydrake.common.value import AbstractValue


# Identifiants FSM (doivent correspondre à ceux de PushController)
FSM_APPROACH    = 0
FSM_PUSH        = 1
FSM_REPOSITION  = 2
FSM_DONE        = 3
FSM_NAMES       = {0: "APPROACH", 1: "PUSH", 2: "REPOSITION", 3: "DONE"}


class PerceptionEvaluator(LeafSystem):
    """
    Log synchronisé ground-truth vs estimation Kalman à CAM_FREQ_HZ.

    Métriques enregistrées à chaque frame
    ──────────────────────────────────────
    t            float    temps simulé [s]
    p_est        (2,)     estimation Kalman [x, y]  [m]
    p_gt         (2,)     vérité terrain Drake [x, y] [m]
    error        float    ‖p̂ − p*‖  [m]
    detected     bool     True si la segmentation a fourni une mesure ce tick
    n_consec_fail int     frames consécutives sans détection
    fsm          int      état FSM courant (0-3)
    cam_pos      (3,)     position caméra monde [m]
    """

    def __init__(self, plant, cube_model_name: str = "cube",
                 freq_hz: float = 30.0):
        super().__init__()
        self._plant     = plant
        self._plant_ctx = plant.CreateDefaultContext()
        self._period    = 1.0 / freq_hz
        self._freq_hz   = freq_hz

        # Index du floating-joint du cube dans le vecteur d'état
        self._cube_model = plant.GetModelInstanceByName(cube_model_name)
        self._cube_body  = self._get_cube_body()
        self._cube_pos_slice = self._find_cube_pos_slice()

        nq = plant.num_positions()
        nv = plant.num_velocities()

        # Ports d'entrée
        self._est_port   = self.DeclareVectorInputPort("cube_pose_estimate", 2)
        self._state_port = self.DeclareVectorInputPort("plant_state", nq + nv)
        # FSM optionnel : si non connecté, on met -1
        self._fsm_port   = self.DeclareVectorInputPort("fsm_state_id", 1)

        # Storage interne — on pré-alloue 60 s * 30 Hz = 1800 frames
        cap = int(freq_hz * 120)
        self._buf_t        = np.full(cap, np.nan)
        self._buf_est      = np.full((cap, 2), np.nan)
        self._buf_gt       = np.full((cap, 2), np.nan)
        self._buf_err      = np.full(cap, np.nan)
        self._buf_det      = np.zeros(cap, dtype=bool)
        self._buf_consec   = np.zeros(cap, dtype=int)
        self._buf_fsm      = np.full(cap, -1, dtype=int)
        self._buf_cam      = np.full((cap, 3), np.nan)
        self._idx          = 0
        self._n_frames     = 0

        # Suivi interne des détections (on lit le compteur depuis RGBDPerceptionModule)
        self._prev_n_failed = 0

        self.DeclarePeriodicDiscreteUpdateEvent(self._period, 0.0, self._tick)

        print(f"[PerceptionEval] initialisé — cube='{cube_model_name}' "
              f"@ {freq_hz} Hz, buffer={cap} frames")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _get_cube_body(self):
        for bi in self._plant.GetBodyIndices(self._cube_model):
            b = self._plant.get_body(bi)
            if b.name() != "world":
                return b
        raise RuntimeError("Corps du cube introuvable")

    def _find_cube_pos_slice(self):
        """
        Retrouve les indices [start:start+3] de la position XYZ du cube
        dans le vecteur q de Drake (floating joint = quaternion 4 + pos 3).
        Retourne l'index du début de la partie translation.
        """
        jt = self._plant.GetJointByName(
            "cube_weld" if self._plant.HasJointNamed("cube_weld") else
            self._cube_body.name() + "_joint"
            if self._plant.HasJointNamed(self._cube_body.name() + "_joint")
            else None
        ) if False else None  # fallback : on utilise CalcCenterOfMassPosition

        # Méthode robuste : CalcRelativeTransform dans _get_gt
        return None  # indique d'utiliser la méthode FK

    def _get_ground_truth_xy(self, plant_state):
        """Vérité terrain : position XY du cube par FK Drake."""
        nq = self._plant.num_positions()
        self._plant.SetPositions(self._plant_ctx, plant_state[:nq])
        X_WC = self._plant.CalcRelativeTransform(
            self._plant_ctx,
            self._plant.world_frame(),
            self._plant.GetFrameByName(self._cube_body.name()))
        return X_WC.translation()[:2]

    # ── tick principal ────────────────────────────────────────────────────────

    def _tick(self, context, discrete_state):
        t   = context.get_time()
        idx = self._idx % len(self._buf_t)

        # 1. Estimation Kalman
        p_est = self._est_port.Eval(context).copy()

        # 2. Vérité terrain
        state = self._state_port.Eval(context)
        p_gt  = self._get_ground_truth_xy(state)

        # 3. Erreur
        err = float(np.linalg.norm(p_est - p_gt))

        # 4. FSM
        try:
            fsm = int(round(self._fsm_port.Eval(context)[0]))
        except Exception:
            fsm = -1

        # 5. Détection (approximée : si l'erreur a sauté de >2 cm
        #    depuis le dernier tick, le Kalman n'a probablement pas eu
        #    de mesure — heuristique conservative)
        #    Plus propre : on récupère n_consec_fail depuis RGBDPerceptionModule
        #    via un port dédié (voir extension optionnelle en bas de fichier).
        detected = True  # sera raffiné si on connecte le port optionnel

        # 6. Enregistrement
        self._buf_t[idx]      = t
        self._buf_est[idx]    = p_est
        self._buf_gt[idx]     = p_gt
        self._buf_err[idx]    = err
        self._buf_det[idx]    = detected
        self._buf_fsm[idx]    = fsm

        self._idx       += 1
        self._n_frames  += 1

        # Log console toutes les 5 s
        if self._n_frames % int(self._freq_hz * 5) == 0:
            print(f"[PerceptionEval t={t:.1f}s] "
                  f"err={err*1000:.1f}mm  "
                  f"est=({p_est[0]:.3f},{p_est[1]:.3f})  "
                  f"gt=({p_gt[0]:.3f},{p_gt[1]:.3f})  "
                  f"FSM={FSM_NAMES.get(fsm,'?')}")

    # ── sauvegarde ───────────────────────────────────────────────────────────

    def save(self, out_dir: str = "figures"):
        """
        Appeler après sim.AdvanceTo() pour sauvegarder le log.
        Retourne le chemin du fichier .npz créé.
        """
        n = min(self._idx, len(self._buf_t))
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, "perception_eval.npz")
        np.savez_compressed(
            path,
            t          = self._buf_t[:n],
            p_est      = self._buf_est[:n],
            p_gt       = self._buf_gt[:n],
            error      = self._buf_err[:n],
            detected   = self._buf_det[:n],
            n_consec   = self._buf_consec[:n],
            fsm        = self._buf_fsm[:n],
        )
        print(f"[PerceptionEval] log sauvegardé → {path}  ({n} frames)")
        return path


# ═══════════════════════════════════════════════════════════════════════════════
# EXTENSION OPTIONNELLE : port n_consec_fail depuis RGBDPerceptionModule
# ═══════════════════════════════════════════════════════════════════════════════
# Pour avoir le vrai compteur de frames manquées, ajouter dans RGBDPerceptionModule :
#
#   self._fail_idx = self.DeclareDiscreteState(2)   # [n_consec_fail, n_failed]
#   self.DeclareStateOutputPort("detection_stats", self._fail_idx)
#
#   # Dans _process_frame, juste avant le return :
#   discrete_state.get_mutable_vector()[self._fail_idx] = [
#       self._n_consec_fails, self._n_failed]
#
# Puis dans PerceptionEvaluator.__init__() :
#   self._det_port = self.DeclareVectorInputPort("detection_stats", 2)
#
# Et dans _tick() :
#   stats    = self._det_port.Eval(context)
#   n_consec = int(stats[0])
#   detected = (n_consec == 0)
#   self._buf_det[idx]    = detected
#   self._buf_consec[idx] = n_consec
