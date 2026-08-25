# Guide d'implémentation — contrôleur sous contraintes + perception RGB-D

Basé sur la lecture de `push_camera.py` (1243 l.) et `benchmark_august.py` (1215 l.).

> **Note :** `camera_model2.py` que tu as uploadé fait 1 octet (vide). Le vrai fichier est `cameramodel2.py` — c'est ce que `push_camera.py` et `benchmark_august.py` importent tous les deux. Je travaille donc à partir de l'interface que j'ai déduite de ses appels : `CameraModel(hz, noise_std_m, freeze_after_first_capture, ...)` et `.sample(t, xy_true, in_contact) -> xy`. Si tu me l'envoies je vérifierai, mais rien de ce qui suit n'en dépend vraiment : le pipeline RGB-D le remplace entièrement.

---

## 0. Points d'ancrage dans ton code

Trois choses que j'ai vérifiées et qui déterminent tout le reste.

**Ta boucle interne est déjà affine en la vitesse commandée.** Fin de `_update()` :

```python
v_cmd   = np.array([v_xy[0], v_xy[1], vz])
F_trans = Kv * (v_cmd - v_sph) + F_floor
send(Jv.T @ F_trans)
```

`F_trans = Kv·(v_cmd − v_sph)` est **linéaire en `v_cmd`**. C'est ce qui rend le cône de frottement — normalement une contrainte conique sur la force — exprimable comme deux inégalités **linéaires** sur ta variable de commande. Le QP est donc trivial, pas approximé. C'est la propriété qui rend tout ce document possible.

**Tes limites de couple sont gérées par écrêtage.** Dans `send()` :

```python
tau = np.clip(tau, -JOINT_TORQUE_LIMITS, JOINT_TORQUE_LIMITS)   # Objective 2
```

Et tu as même un print de diagnostic qui liste les articulations saturées, donc tu sais que ça arrive. L'écrêtage change la *direction* du vecteur de couple, pas seulement sa norme : le robot n'applique alors plus la force que le contrôleur croit appliquer. Une contrainte QP conserve la direction et sacrifie la norme, ce qui est le bon compromis. C'est un argument concret à faire en défense.

**`c_sq` est mort, et ton propre code le dit.** Ligne ~305 :

```python
self.c_sq = get_c_squared(shape_type, half_extent, self.half_extent_y)   # diagnostic only
```

**Un point qui bloque l'analyse de glissement.** Ton `_force_log` ne garde que la norme :

```python
self._force_log.append({"t": t, "f_mag": f_mag, "drake_ok": drake_ok, ...})
```

Sans le vecteur, impossible de décomposer en f_n / f_t, donc impossible de mesurer le glissement en post-traitement pur. Je m'étais avancé là-dessus au message précédent — correction : il te faut 3 lignes de log en plus, puis un re-run. C'est du temps machine, pas du temps humain, mais ce n'est pas gratuit.

```python
self._force_log.append({
    "t": t, "f_mag": f_mag, "drake_ok": drake_ok, "contact": bool(self._contact),
    "fx": float(self._fk_filt[0]), "fy": float(self._fk_filt[1]),      # ← ajouter
    "nx": float(self._face_normal[0]), "ny": float(self._face_normal[1]),  # ← ajouter
    "v_obj_mag": ..., "v_obj_along": ...,
})
```

Puis en post-traitement, avec `n_push = -face_normal` (la normale entrante, dans le sens de la poussée) :

```python
n_push = -np.array([nx, ny])
t_c    = np.array([-n_push[1], n_push[0]])
f_n    = f @ n_push
f_t    = f @ t_c
slip   = np.abs(f_t) > mu_c * np.maximum(f_n, 1e-6)
```

`mu_c` doit être le **coefficient combiné** que Drake utilise réellement (moyenne harmonique, ton Eq. 4.21), pas le `mu` du SDF — sinon tu mesureras un glissement fantôme.

**Détail mineur, sans impact sur ton benchmark.** `run_trial()` a un argument par défaut mutable :

```python
def run_trial(..., camera_model=CameraModel(freeze_after_first_capture=None, noise_std_m=0.003)):
```

Cet objet est instancié une seule fois à l'import et partagé par tous les appels qui ne passent pas `camera_model` explicitement — son état interne (dernière capture, valeur gelée) fuit d'un run à l'autre. Ton benchmark n'est **pas** touché : il crée une instance fraîche par run à la ligne 442 et la passe explicitement. Mais tes runs manuels/interactifs le sont. Mets `camera_model=None` par défaut et instancie dans le corps.

---

# Partie A — Le contrôleur sous contraintes

## A.1 Stratégie : deux étapes, la première sans risque

Ne remplace pas ta loi d'un coup. Fais-la d'abord **filtrer**.

**Étape 1 — le QP comme filtre de sécurité.** Ta loi heuristique calcule `v_xy` comme aujourd'hui. Le QP prend ce `v_xy` comme référence et le projette sur l'ensemble admissible :

```
min ‖v − v_xy‖² + ρ‖s‖²   s.c.  C1…C6
```

Si `v_xy` est déjà admissible, la solution est `v = v_xy` et **rien ne change**. Le contrôleur se comporte exactement comme avant. Tu ne peux donc rien casser.

Ce que tu gagnes immédiatement : **la fraction de ticks où `v ≠ v_xy` est le taux de violation de contraintes.** C'est ta figure headline, et tu l'obtiens du même run qui applique déjà la correction. Étape 1 est à la fois la mesure du problème et sa solution.

**Étape 2 — le QP devient le contrôleur.** Une fois l'étape 1 validée, tu remplaces la référence par quelque chose de propre (`v_ref = v_profile · d̂`) et tu ajoutes le terme de yaw. Les heuristiques disparaissent : `V_OBJ_DAMPING`, la saturation d'admittance `F_MAX/f_mag`, le plafond dur `V_XY_HARD_CAP` et `K_LAT` sont tous subsumés par C1–C6.

## A.2 Formulation

Variable de décision `v ∈ R²` (remplace `v_xy`), plus deux variables d'écart `s ∈ R²₊`.

Avec `n_push = −self._face_normal` (unitaire, du contact vers l'intérieur de l'objet) et `t_c = [−n_push[1], n_push[0]]`, la force planaire prédite par ta boucle d'impédance est :

```
f(v) = Kv·(v − v_sph[:2])
f_n(v) = aₙ·v + bₙ    avec  aₙ = Kv·n_push ,  bₙ = −Kv·(n_push · v_sph[:2])
f_t(v) = a_t·v + b_t   avec  a_t = Kv·t_c   ,  b_t = −Kv·(t_c · v_sph[:2])
```

**Remarque importante pour ta défense : `f(v)` ne dépend que de l'état du robot** (`v_sph`, issu de `Jv @ v_arm`). Le QP n'interroge ni `ContactResults`, ni la position vraie de l'objet. Seule `n_push` vient de la géométrie — et pour le cylindre elle est purement construite depuis `d_hat`. Ton contrôleur sous contraintes est donc **moins dépendant de la vérité terrain que ta loi actuelle**, qui utilise `self._fk_filt` issu de `ContactResults`.

| # | Contrainte | Forme |
|---|---|---|
| C1 | Cône de frottement (souple) | `±(a_t·v + b_t) − μ_c(aₙ·v + bₙ) ≤ s₀` |
| C2 | Maintien du contact (souple) | `aₙ·v + bₙ ≥ F_MIN − s₁` |
| C3 | Plafond de force (dure) | `aₙ·v + bₙ ≤ F_MAX` |
| C5 | Couples articulaires (dure) | `\|Kv·Jv[:2,:]ᵀ·v + b_τ\| ≤ τ_max` |
| C6 | Vitesse / freinage (dure) | octogone : `u_k·v ≤ v_profile`, k = 0…7 |

C4 (vitesses articulaires) est optionnelle — ajoute-la seulement si tu observes des `q̇` élevés.

## A.3 Code

```python
from pydrake.solvers import MathematicalProgram, Solve

RHO_SLACK = 1e4          # pénalité des contraintes souples
OCTAGON = np.array([[np.cos(k*np.pi/4), np.sin(k*np.pi/4)] for k in range(8)])


def _push_constraint_data(self, v_sph, Jv, g_arm, f_z, mu_c):
    """Coefficients affines communes au check rapide et au QP."""
    n_push = -self._face_normal
    t_c    = np.array([-n_push[1], n_push[0]])
    a_n, b_n = Kv * n_push, -Kv * float(n_push @ v_sph[:2])
    a_t, b_t = Kv * t_c,    -Kv * float(t_c    @ v_sph[:2])
    A_tau = Kv * Jv[:2, :].T                                     # (7, 2)
    b_tau = -Kv * (Jv[:2, :].T @ v_sph[:2]) + Jv[2, :].T * f_z - g_arm
    return dict(a_n=a_n, b_n=b_n, a_t=a_t, b_t=b_t,
                A_tau=A_tau, b_tau=b_tau, mu_c=mu_c)


def _violates(self, v, cd, v_cap):
    """Vérification numpy pure, ~10 µs. Sert de garde ET de compteur."""
    f_n = float(cd["a_n"] @ v + cd["b_n"])
    f_t = float(cd["a_t"] @ v + cd["b_t"])
    if abs(f_t) > cd["mu_c"] * max(f_n, 1e-9):        return "cone"
    if f_n < F_MIN - 1e-9:                            return "contact"
    if f_n > F_MAX + 1e-9:                            return "fmax"
    if np.linalg.norm(v) > v_cap + 1e-9:              return "speed"
    tau = cd["A_tau"] @ v + cd["b_tau"]
    if np.any(np.abs(tau) > JOINT_TORQUE_LIMITS + 1e-9):  return "torque"
    return None


def _solve_push_qp(self, v_ref, cd, v_cap):
    prog = MathematicalProgram()
    v = prog.NewContinuousVariables(2, "v")
    s = prog.NewContinuousVariables(2, "s")

    a_n, b_n, a_t, b_t, mu = cd["a_n"], cd["b_n"], cd["a_t"], cd["b_t"], cd["mu_c"]

    # C1 — cône de frottement (souple)
    prog.AddLinearConstraint(( a_t - mu*a_n) @ v <= s[0] - ( b_t - mu*b_n))
    prog.AddLinearConstraint((-a_t - mu*a_n) @ v <= s[0] - (-b_t - mu*b_n))
    # C2 — maintien du contact (souple)
    prog.AddLinearConstraint(a_n @ v >= F_MIN - s[1] - b_n)
    # C3 — plafond de force (dure)
    prog.AddLinearConstraint(a_n @ v <= F_MAX - b_n)
    # C6 — vitesse (octogone, dure)
    for u in OCTAGON:
        prog.AddLinearConstraint(u @ v <= v_cap)
    # C5 — couples (dure)
    prog.AddLinearConstraint(cd["A_tau"],
                             -JOINT_TORQUE_LIMITS - cd["b_tau"],
                              JOINT_TORQUE_LIMITS - cd["b_tau"], v)

    prog.AddBoundingBoxConstraint(0.0, np.inf, s)
    prog.AddQuadraticErrorCost(np.eye(2), v_ref, v)
    prog.AddQuadraticCost(RHO_SLACK * (s @ s), is_convex=True)

    res = Solve(prog)
    if not res.is_success():
        self._n_qp_fail += 1
        return v_ref, False            # repli : on garde l'heuristique
    return res.GetSolution(v), True
```

Branchement, en remplacement des trois blocs `V_OBJ_DAMPING` / saturation d'admittance / `V_XY_HARD_CAP` :

```python
f_z = float(Kv * (vz - v_sph[2]) + F_floor[2])
cd  = self._push_constraint_data(v_sph, Jv, g_arm, f_z, self._mu_combined)

self._n_ticks += 1
reason = self._violates(v_xy, cd, v_profile)
if reason is not None:
    self._viol_counts[reason] = self._viol_counts.get(reason, 0) + 1
    v_xy, ok = self._solve_push_qp(v_xy, cd, v_profile)
```

## A.4 Le piège du 1 kHz

`_update` tourne à `1/1000`. **Construire un `MathematicalProgram` en Python coûte 200–500 µs**, largement plus que le solveur lui-même (~30 µs pour 2 variables). À 1 kHz, tu passerais 30 % du temps mural dans la construction du problème, et ton benchmark de 48 runs × 25 s exploserait de plusieurs heures.

C'est exactement pourquoi le `_violates()` ci-dessus est écrit en numpy pur : **il filtre.** Le QP n'est construit que sur les ticks effectivement en violation. Si ceux-ci sont minoritaires (mon hypothèse), le surcoût est négligeable, et si ils sont majoritaires, tu viens de démontrer que le problème est massif — dans les deux cas l'information est bonne.

Si le surcoût reste trop élevé, décime : ne résous que tous les 5 ticks (200 Hz) et tiens la solution entre deux. La géométrie de contact ne bouge pas en 1 ms, c'est physiquement justifiable et ça se défend en une phrase.

## A.5 Faisabilité

C1 + C2 + C5 peuvent entrer en conflit — typiquement en fin de course, quand le freinage veut ralentir alors que le maintien du contact veut pousser. D'où la hiérarchie :

- **Dures** (jamais relâchées) : C3, C5, C6 — sécurité matérielle et intégrité de la simulation.
- **Souples** (variables d'écart, pénalité `RHO_SLACK`) : C1, C2 — désirables pour la tâche, pas critiques.

Logge `s` à chaque solve. `s[0] > 0` signifie « la tâche demandée exigeait de faire glisser le pousseur » — c'est-à-dire que **le push demandé est mécaniquement infaisable à ce coefficient de frottement**. Cette information n'existe nulle part dans ton contrôleur actuel, et c'est une des choses les plus intéressantes que le QP produit.

## A.6 Étape 2 — le terme de yaw

Une fois l'étape 1 stable, remplace `v_ref` :

```python
v_ref = v_profile * d_hat        # plus de push_dir_corrected, plus de K_LAT
```

et ajoute au coût la régulation du yaw, qui absorbe le centrage latéral :

```python
# p_y : décalage latéral du pousseur sur la face, repère face
p_y   = float(t_c @ (p_sph[:2] - object_xy))
K_th  = k_theta * mass * self.c_sq          # ← c_sq entre enfin dans la boucle
tau_cmd = K_th * theta_err - K_om * omega_o
p_y_des = np.clip(-tau_cmd / max(f_n_meas, F_MIN), -0.6*a, 0.6*a)
vpy_des = K_PY * (p_y_des - p_y)

# coût additionnel : w_theta * (t_c·(v − v_obj) − vpy_des)²
prog.AddQuadraticCost(w_theta * ((t_c @ v - float(t_c @ self._v_obj_filt) - vpy_des)**2),
                      is_convex=True)
```

`theta_err` a besoin du yaw **mesuré**, pas de `_get_object_yaw(q)` qui lit le quaternion Drake. C'est la dépendance qui impose de faire la Partie B d'abord — le fit de plan sur le nuage RGB-D te le donne pour le cube.

---

# Partie B — Perception RGB-D

## B.1 Architecture : ne rends jamais à 1 kHz

Ton `CameraModel` actuel est appelé depuis `_update`, à 1 kHz, et c'est sans conséquence parce qu'il ne fait qu'ajouter du bruit gaussien. Un rendu VTK coûte **10–30 ms**. Appelé à 1 kHz sur 25 s de simulation, ça fait 25 000 rendus par run, soit ~8 minutes de rendu **par run**, ~7 heures pour la grille de 48. Avec des répétitions, c'est des semaines.

**Solution : un `LeafSystem` de perception séparé, avec sa propre mise à jour périodique à 30 Hz.** Le rendu n'est déclenché que par `Eval()` sur les ports image, donc uniquement dans son callback. 750 rendus par run, ~15 s. C'est la différence entre faisable et infaisable, et c'est aussi la structure Drake idiomatique.

## B.2 Ajout de la caméra dans `create_sim_scene`

```python
from pydrake.geometry import (MakeRenderEngineVtk, RenderEngineVtkParams,
                              ClippingRange, DepthRange, RenderCameraCore,
                              ColorRenderCamera, DepthRenderCamera)
from pydrake.systems.sensors import RgbdSensor, CameraInfo

RENDERER = "vtk"

def make_camera_pose(eye, target):
    """Convention optique Drake : +z avant, +x droite, +y bas."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye;  z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0]);  x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return RigidTransform(RotationMatrix(np.column_stack([x, y, z])), eye)

# --- dans create_sim_scene, APRÈS plant.Finalize() ---
sg.AddRenderer(RENDERER, MakeRenderEngineVtk(RenderEngineVtkParams()))

intrinsics = CameraInfo(width=640, height=480, fov_y=np.pi/4)
core = RenderCameraCore(RENDERER, intrinsics,
                        ClippingRange(0.05, 5.0), RigidTransform())   # X_BS = I → C ≡ B
color_cam = ColorRenderCamera(core, show_window=False)
depth_cam = DepthRenderCamera(core, DepthRange(0.05, 4.0))

X_WC = make_camera_pose(eye=CAMERA_EYE, target=CAMERA_TARGET)
cam = builder.AddNamedSystem("rgbd", RgbdSensor(
    parent_id=sg.world_frame_id(), X_PB=X_WC,
    color_camera=color_cam, depth_camera=depth_cam))
builder.Connect(sg.get_query_output_port(), cam.query_object_input_port())
```

`X_BS = RigidTransform()` dans le `RenderCameraCore` fait coïncider le repère optique C avec le repère corps B, donc `X_WC = X_PB`. Ça t'évite la principale source de bugs de convention.

**Placement.** Le choix est une décision de conception à justifier dans le mémoire, pas un détail. Un compromis raisonnable pour tes positions (objet vers x≈0.30–0.35, cible vers x≈0.55) :

```python
CAMERA_EYE    = [1.30, -0.55, 0.85]   # de côté, en hauteur, ~40° d'inclinaison
CAMERA_TARGET = [0.45,  0.10, 0.05]   # milieu du couloir de poussée
```

De côté plutôt que derrière le robot : le bras occulte alors *partiellement* pendant le contact — ce qui est le phénomène que tu veux étudier — au lieu de tout masquer. Vérifie visuellement une fois avec `show_window=True` avant de lancer quoi que ce soit.

## B.3 Le système de perception

```python
from pydrake.systems.sensors import ImageRgba8U, ImageDepth32F, ImageLabel16I

MIN_PIXELS = 60

class RgbdPerception(LeafSystem):
    """Remplace CameraModel. Sortie : [x, y, valid]."""

    def __init__(self, intrinsics, X_WC, shape_type, half_extent,
                 fps=30.0, latency_s=0.08):
        super().__init__()
        self._color = self.DeclareAbstractInputPort("color", AbstractValue.Make(ImageRgba8U()))
        self._depth = self.DeclareAbstractInputPort("depth", AbstractValue.Make(ImageDepth32F()))
        self._label = self.DeclareAbstractInputPort("label", AbstractValue.Make(ImageLabel16I()))

        self._fx, self._fy = intrinsics.focal_x(), intrinsics.focal_y()
        self._cx, self._cy = intrinsics.center_x(), intrinsics.center_y()
        self._R_WC = X_WC.rotation().matrix()
        self._p_WC = X_WC.translation()
        self.shape_type, self.half_extent = shape_type, half_extent
        self._latency = latency_s

        self._buffer   = []      # [(t_capture, xy)]
        self.vis_log   = []      # [(t, n_pixels)]  → figure d'occlusion
        self.iou_log   = []      # [(t, IoU)]       → validation segmentation
        self.yaw_est   = None

        idx = self.DeclareDiscreteState(3)
        self.DeclareStateOutputPort("object_xy_est", idx)
        self.DeclarePeriodicDiscreteUpdateEvent(1.0 / fps, 0.0, self._capture)

    # ── segmentation couleur (vraie opération de vision) ──────────────
    def _segment(self, rgba):
        r = rgba[:, :, 0].astype(np.int16)
        g = rgba[:, :, 1].astype(np.int16)
        b = rgba[:, :, 2].astype(np.int16)
        return (r > 120) & (g < 80) & (b < 80)      # objet peint en rouge saturé

    def _deproject(self, depth, mask):
        vs, us = np.nonzero(mask)
        d = depth[vs, us]
        p_C = np.stack([(us - self._cx) * d / self._fx,
                        (vs - self._cy) * d / self._fy,
                        d], axis=1)
        return p_C @ self._R_WC.T + self._p_WC

    def _capture(self, context, discrete_state):
        t = context.get_time()
        rgba  = self._color.Eval(context).data
        depth = self._depth.Eval(context).data[:, :, 0]

        mask  = self._segment(rgba) & np.isfinite(depth) & (depth > 0)
        n_px  = int(mask.sum())
        self.vis_log.append((t, n_px))

        if n_px >= MIN_PIXELS:
            pts = self._deproject(depth, mask)
            xy  = self._estimate_center(pts[:, :2])
            self._buffer.append((t, xy))

        # latence : on publie la mesure la plus récente déjà "arrivée"
        ready = [(tc, xy) for tc, xy in self._buffer if tc + self._latency <= t]
        st = discrete_state.get_mutable_vector()
        if ready:
            _, xy = ready[-1]
            st.SetFromVector([xy[0], xy[1], 1.0])
        else:
            st.SetFromVector([0.0, 0.0, 0.0])
```

## B.4 Le piège qui fera tout échouer si tu l'ignores

**Le centroïde du nuage déprojeté n'est pas le centre de l'objet.** Une caméra unique ne voit qu'une face. Pour ton cylindre de rayon R = 0.1 m, le centroïde de l'arc visible est à 2R/π ≈ **63.7 mm** du centre, vers la caméra. Pour ton cube de demi-côté a = 0.05 m vu de face, la face visible est à **50 mm** du centre.

Ton `DONE_THRESHOLD` vaut **25 mm**. Le biais est donc 2 à 2.5 fois le seuil de succès, et il est **systématique** — aucun filtrage, aucun moyennage ne le réduira. Si tu branches un centroïde naïf, tes 48 runs échoueront tous et tu perdras une journée à chercher le bug dans le contrôleur alors qu'il est dans le pipeline.

C'est aussi, dit autrement, la raison pour laquelle cette correction **est** la contribution de perception. Personne ne peut te reprocher de l'avoir sous-traitée au simulateur.

```python
def _estimate_center(self, pts_xy):
    centroid = pts_xy.mean(axis=0)
    u = centroid - self._p_WC[:2]          # caméra → objet, dans le plan
    u /= np.linalg.norm(u) + 1e-9

    if self.shape_type == "cylinder":
        R  = self.half_extent
        c0 = centroid + (2.0 / np.pi) * R * u          # correction analytique
        return self._fit_circle_known_R(pts_xy, c0, R)  # raffinement

    # cube : la face verticale se projette en un segment dans le plan xy.
    # PCA → direction du segment ; la normale donne le yaw ET la correction.
    q = pts_xy - centroid
    _, _, Vt = np.linalg.svd(q, full_matrices=False)
    tangent = Vt[0]
    n = np.array([-tangent[1], tangent[0]])
    if n @ u > 0:
        n = -n                                          # normale vers la caméra
    self.yaw_est = float(np.arctan2(n[1], n[0]))        # ← alimente le terme de yaw du QP
    return centroid - self.half_extent * n

@staticmethod
def _fit_circle_known_R(pts, c0, R, iters=8):
    """Gauss-Newton sur min Σ(‖pᵢ−c‖ − R)², rayon connu, 2 inconnues."""
    c = c0.copy()
    for _ in range(iters):
        d = pts - c
        rho = np.linalg.norm(d, axis=1) + 1e-12
        J = -d / rho[:, None]                # ∂rᵢ/∂c
        r = rho - R
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        c += step
        if np.linalg.norm(step) < 1e-6:
            break
    return c
```

Le cube te donne le **yaw mesuré** en prime, gratuitement, via la normale du segment. C'est ce dont l'étape 2 du QP a besoin, et c'est une observabilité que tu n'as aujourd'hui nulle part.

## B.5 Branchement sur le contrôleur

Trois modifications dans `PushController` :

```python
# __init__
self._perception_port = self.DeclareVectorInputPort("object_xy_est", 3)
self._last_percept = None

# _update, en remplacement du bloc camera_model
if self._use_perception:
    est = self._perception_port.Eval(context)
    if est[2] > 0.5:
        self._last_percept = est[:2].copy()
    object_xy = (self._last_percept.copy() if self._last_percept is not None
                 else object_xy_true.copy())
else:
    object_xy = object_xy_true.copy()
```

et dans `create_sim_scene` :

```python
percep = builder.AddNamedSystem("perception", RgbdPerception(
    intrinsics, X_WC, shape_type, half_extent))
builder.Connect(cam.color_image_output_port(),    percep.GetInputPort("color"))
builder.Connect(cam.depth_image_32F_output_port(), percep.GetInputPort("depth"))
builder.Connect(cam.label_image_output_port(),     percep.GetInputPort("label"))
builder.Connect(percep.GetOutputPort("object_xy_est"),
                ctrl.GetInputPort("object_xy_est"))
```

Et dans `write_sdf()` de `benchmark_august.py`, donne à l'objet une couleur saturée cohérente avec `_segment()` :

```xml
<visual name="visual">
  <geometry>...</geometry>
  <material><diffuse>0.9 0.1 0.1 1.0</diffuse></material>
</visual>
```

## B.6 Validation — la section que le mémoire n'a pas

Trois mesures, toutes produites par le pipeline sans travail supplémentaire :

1. **IoU de segmentation** — compare ton masque couleur à l'image de labels. **L'image de labels ne doit jamais alimenter le contrôleur**, uniquement l'évaluation : c'est ce qui distingue une validation d'un oracle. Dis-le explicitement dans le mémoire, le jury le remarquera.
2. **Erreur de position** en fonction de la distance caméra, de l'obliquité, et du nombre de pixels visibles. Avec et sans la correction de biais de B.4 — la comparaison des deux courbes *est* la figure qui justifie la contribution.
3. **Chronologie d'occlusion** — `vis_log` te donne le nombre de pixels visibles au cours du push. C'est ce qui remplace ton `p = 0.9`. Montre que l'occlusion réelle est corrélée dans le temps (longues rafales pendant le contact) là où un tirage de Bernoulli produit des pertes isolées. Un modèle i.i.d. sous-estime structurellement la durée maximale sans mesure, et c'est cette durée qui pilote l'erreur de dead reckoning.

---

# Ordre d'exécution

```
1. Log de f_n/f_t (3 lignes)  →  re-run  →  taux de glissement
                                              └─ justifie A avant de l'écrire
2. Partie A étape 1 : QP filtre de sécurité      (~2 j)
3. Partie B : pipeline RGB-D                     (~3 j)
4. Partie A étape 2 : terme de yaw (a besoin de B) (~2 j)
5. Répétitions + baselines B2/B3/B4              (temps machine)
```

L'étape 1 avant la Partie B, malgré ce que je disais au message précédent : le QP ne dépend pas de la perception (voir A.2), donc il est autonome, et l'étape 1 est sans risque. Le terme de yaw, lui, attend B.

# Les cinq pièges, en résumé

1. **Ne construis pas un `MathematicalProgram` à 1 kHz.** Filtre avec `_violates()` en numpy.
2. **Ne rends pas à 30 Hz depuis `_update`.** `LeafSystem` séparé avec son propre événement périodique.
3. **Le biais de centroïde vaut 2 à 2.5 × ton seuil de succès.** Corrige-le avant de conclure quoi que ce soit.
4. **`mu_c` est le coefficient combiné de Drake** (moyenne harmonique), pas celui du SDF.
5. **L'image de labels ne sert qu'à valider**, jamais à alimenter le contrôleur.
