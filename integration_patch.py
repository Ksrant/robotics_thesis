"""
integration_patch.py
═══════════════════════════════════════════════════════════════════════════════
Ce fichier montre exactement les 4 petites modifications à faire dans
push_perception_force.py pour brancher PerceptionEvaluator.

Cherche chaque bloc "AVANT" et remplace-le par le bloc "APRÈS".
"""

# ══════════════════════════════════════════════════════════════════════════════
# MODIFICATION 1 — Import en tête de fichier
# ══════════════════════════════════════════════════════════════════════════════

# AVANT (rien à cet endroit, juste après les autres imports) :
# from helper.dynamics import CalcRobotDynamics

# APRÈS : ajouter la ligne suivante juste en dessous
# from perception_evaluator import PerceptionEvaluator


# ══════════════════════════════════════════════════════════════════════════════
# MODIFICATION 2 — Exposer l'état FSM depuis PushController
# ══════════════════════════════════════════════════════════════════════════════

# Dans PushController.__init__(), après la ligne :
#   state_idx = self.DeclareDiscreteState(nv)

# AVANT :
#   state_idx = self.DeclareDiscreteState(nv)
#   self.DeclareStateOutputPort("tau_u", state_idx)
#   self.DeclarePeriodicDiscreteUpdateEvent(1/1000, 0.0, self._update)

# APRÈS :
#   state_idx = self.DeclareDiscreteState(nv)
#   self.DeclareStateOutputPort("tau_u", state_idx)
#
#   # --- Exposition de l'état FSM pour PerceptionEvaluator ---
#   FSM_STATE_IDS = {STATE_APPROACH: 0, STATE_PUSH: 1,
#                   STATE_REPOSITION: 2, STATE_DONE: 3}
#   self._FSM_STATE_IDS = FSM_STATE_IDS
#   self._fsm_out_idx = self.DeclareDiscreteState(1)
#   self.DeclareStateOutputPort("fsm_state_id", self._fsm_out_idx)
#   # ----------------------------------------------------------
#
#   self.DeclarePeriodicDiscreteUpdateEvent(1/1000, 0.0, self._update)


# ══════════════════════════════════════════════════════════════════════════════
# MODIFICATION 3 — Écrire l'ID FSM à chaque tick dans _update()
# ══════════════════════════════════════════════════════════════════════════════

# Dans PushController._update(), TOUT EN BAS, juste avant le dernier
# discrete_state.get_mutable_vector().SetFromVector(tau_total) de la phase PUSH.
# Plus simple : mettre la ligne UNE SEULE FOIS, à la toute fin de _update(),
# après le SetFromVector(tau_total) terminal (il y en a plusieurs à cause des
# early-returns — copier cette ligne dans chacun des blocs return).

# EXEMPLE pour le bloc PUSH terminal :

# AVANT :
#   discrete_state.get_mutable_vector().SetFromVector(tau_total)

# APRÈS (dans CHAQUE bloc qui appelle SetFromVector, ajouter juste avant) :
#   # Mise à jour de l'ID FSM pour PerceptionEvaluator
#   fsm_id = self._FSM_STATE_IDS.get(self._ctrl_state, -1)
#   self._fsm_out_idx_state = discrete_state.get_mutable_vector()
#   # Note : on ne peut pas écrire dans _fsm_out_idx directement depuis
#   # discrete_state car les deux états sont dans le même vecteur discret.
#   # Solution propre : utiliser un DeclareAbstractState ou un port de publication.
#   # Solution simple ci-dessous (un seul vecteur discret étendu) :

# ── SOLUTION PLUS SIMPLE (recommandée) ────────────────────────────────────────
# Plutôt que de toucher chaque return, utiliser DeclarePeriodicPublishEvent
# séparé uniquement pour le FSM state :
#
#   # Dans __init__() :
#   self._fsm_cache = [0]   # liste mutable — partagée entre méthodes
#   fsm_idx = self.DeclareDiscreteState(1)
#   self.DeclareStateOutputPort("fsm_state_id", fsm_idx)
#   self._fsm_state_port_idx = fsm_idx
#   self.DeclarePeriodicDiscreteUpdateEvent(1/1000, 0.0, self._update_fsm_out)
#
#   # Nouvelle méthode (à ajouter dans PushController) :
#   def _update_fsm_out(self, context, discrete_state):
#       FSM_MAP = {STATE_APPROACH: 0, STATE_PUSH: 1,
#                  STATE_REPOSITION: 2, STATE_DONE: 3}
#       discrete_state.get_mutable_vector().SetFromVector(
#           np.array([FSM_MAP.get(self._ctrl_state, -1)], dtype=float))
#
# Cette approche est propre et ne touche pas à _update().


# ══════════════════════════════════════════════════════════════════════════════
# MODIFICATION 4 — Brancher dans create_sim_scene() et run_simulation()
# ══════════════════════════════════════════════════════════════════════════════

# Dans create_sim_scene(), après la ligne :
#   builder.Connect(plant.get_contact_results_output_port(),
#                   ctrl.GetInputPort("contact_results"))

# APRÈS (ajouter ces lignes) :
#
#   # ── PerceptionEvaluator ─────────────────────────────────────────────────
#   evaluator = builder.AddNamedSystem("PerceptionEval",
#       PerceptionEvaluator(plant, cube_model_name="cube",
#                           freq_hz=CAM_FREQ_HZ))
#   builder.Connect(perception.GetOutputPort("cube_pose_estimate"),
#                   evaluator.GetInputPort("cube_pose_estimate"))
#   builder.Connect(plant.get_state_output_port(),
#                   evaluator.GetInputPort("plant_state"))
#   builder.Connect(ctrl.GetOutputPort("fsm_state_id"),
#                   evaluator.GetInputPort("fsm_state_id"))
#   # ────────────────────────────────────────────────────────────────────────

# Et modifier le return pour exposer evaluator :
# AVANT :
#   return builder.Build(), logger, best_q7

# APRÈS :
#   return builder.Build(), logger, best_q7, evaluator


# ══════════════════════════════════════════════════════════════════════════════
# MODIFICATION 5 — Sauvegarder et analyser après la simulation
# ══════════════════════════════════════════════════════════════════════════════

# Dans run_simulation(), modifier :
# AVANT :
#   diagram, logger, best_q7 = create_sim_scene(sim_time_step)

# APRÈS :
#   diagram, logger, best_q7, evaluator_sys = create_sim_scene(sim_time_step)

# Et après sim.AdvanceTo(60.0), ajouter :
#
#   # Sauvegarde du log de perception
#   evaluator = diagram.GetSubsystemByName("PerceptionEval")
#   npz_path  = evaluator.save("figures")
#
#   # Analyse post-run automatique
#   from analyse_perception import analyse_single
#   analyse_single(npz_path, out_dir="figures")
