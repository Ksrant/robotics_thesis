# Guide 2 — Le contrôleur sous contraintes, formulation articulaire

Ce guide remplace la Partie A du guide précédent. Le feedback change la cible : un QP en espace tâche ne répond qu'à la moitié des reproches.

---

## Pourquoi changer de formulation

Le QP en vitesse que je t'avais donné impose le cône de frottement et les limites de couple. Mais trois commentaires du superviseur portent sur des choses qu'il ne traite **pas** :

| Commentaire | QP tâche (guide 1) | QP articulaire |
|---|---|---|
| p32 — « The Panda has 7 DoF, yet only the translational task is controlled. What happens to the remaining redundancy/null-space dynamics? » | ✗ non traité | ✓ résolu par le coût de posture |
| p29 — « I do not see joint-velocity limiting/constraint handling » | ~ indirect via Jᵀ | ✓ contrainte directe sur q̇ |
| p29 — « no constrained contact-force optimisation is solved » | ~ force implicite | ✓ **f est une variable de décision** |

Le troisième point est le plus important. Il demande une *optimisation de force de contact sous contraintes*. Dans le QP tâche, la force est une conséquence affine de la vitesse ; dans le QP articulaire, **c'est une variable que tu optimises explicitement, sous son cône de frottement**. C'est littéralement ce qu'il demande, et la différence est visible au premier coup d'œil sur la formulation.

Bonus : cette formulation te donne de la littérature à citer, ce qui manque cruellement à ton Ch. 5 — *task-space inverse dynamics* / *whole-body QP control* (Escande et al., Del Prete et al., Herzog et al., Feng et al.). Ton chapitre passe d'« empilement d'heuristiques maison » à « instance d'un cadre établi, spécialisé à la poussée non préhensile ».

---

## La formulation

**Variables de décision** (19 au total) :

```
q̈  ∈ R⁷     accélérations articulaires
τ  ∈ R⁷     couples articulaires
f  ∈ R³     force de contact appliquée PAR l'effecteur SUR l'objet
s  ∈ R²₊    variables d'écart (contraintes souples)
```

**Contrainte d'égalité — la dynamique.** Attention à la convention de signe, c'est exactement le commentaire p31 :

> Drake utilise `M(q)v̇ + C(q,v)v = τ_g(q) + τ_app`, avec `τ_g = CalcGravityGeneralizedForces()` **du côté droit**. Ton Eq. (4.1) écrit `M q̈ + C q̇ + g(q) = τ + τ_ext`, avec `g(q)` **à gauche**. Les deux sont cohérents avec `g(q) = −τ_g`, mais ton mémoire ne le dit nulle part — d'où la remarque. Ton code, lui, est correct (`tau = tau_xyz - g_arm` donne bien la compensation attendue). Il te suffit d'ajouter une phrase de convention au Ch. 4.

La réaction sur le robot est `−f` (commentaire p27, que cette formulation corrige structurellement) :

```
M(q)·q̈ − τ + Jᵥᵀ·f  =  τ_g(q) − C(q,q̇)q̇
```

**Contraintes d'inégalité :**

| # | Contrainte | Forme | Répond à |
|---|---|---|---|
| I1 | Couples | `\|τ\| ≤ τ_max` | p32 (remplace `np.clip`) |
| I2 | Vitesses articulaires | `\|q̇ + q̈·dt\| ≤ q̇_max` | p29 |
| I3 | Positions articulaires | `q̈ ≤ 2(q_max − q − q̇·dt)/dt²` (et sym.) | p29, §4.2.1 |
| I4 | Cône de frottement | `\|f·t̂\| ≤ μ_c (f·n̂) + s₀` | p29, Eq. (4.20), Lynch–Mason [5] |
| I5 | Maintien du contact | `f·n̂ ≥ F_min − s₁` | — |
| I6 | Plafond de force | `f·n̂ ≤ F_max` | — |
| I7 | Hors contact | `f = 0` quand la FSM n'est pas en contact | mode hybride |

**Coût :**

```
w_task · ‖Jᵥ q̈ + J̇ᵥq̇ − a_des‖²        suivi de la tâche cartésienne (3 dim)
+ w_post · ‖q̈ − q̈_post‖²                posture dans l'espace nul (4 dim)  ← p32
+ w_f    · ‖f − f_des‖²                  force de contact désirée
+ w_reg  · ‖τ‖²                          régularisation
+ ρ      · ‖s‖²                          pénalité des contraintes souples
```

avec

```
a_des   = K_a · (v_des − v_ee)                  v_des vient de ta boucle externe
q̈_post = K_p·(q_post − q) − K_d·q̇              posture de repli, loin des butées
```

**C'est le terme `w_post` qui répond à p32.** Ta tâche occupe 3 dimensions, ton robot en a 7 : les 4 restantes sont aujourd'hui laissées à la dynamique du système, sans que tu en dises rien. Ici elles sont explicitement résolues par une tâche secondaire, hiérarchisée par les poids. C'est une réponse en une phrase à une question à laquelle ton mémoire n'en a aucune.

---

## Le code

```python
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.multibody.tree import JacobianWrtVariable

NQ_ARM = 7
W_TASK, W_POST, W_F, W_REG, RHO = 1e3, 1e-1, 1e0, 1e-4, 1e5


def _dynamics_terms(self):
    """M, C·q̇, τ_g, Jᵥ, J̇ᵥq̇ restreints aux articulations du bras."""
    ctx, plant = self.plant_context_ad, self.plant
    idx = self._arm_vel_idx
    M    = plant.CalcMassMatrix(ctx)[np.ix_(idx, idx)]
    Cv   = plant.CalcBiasTerm(ctx)[idx]
    tau_g = plant.CalcGravityGeneralizedForces(ctx)[idx]
    ee   = plant.GetFrameByName("panda_hand")
    Jv   = plant.CalcJacobianTranslationalVelocity(
               ctx, JacobianWrtVariable.kV, ee, FINGER_TIP_OFFSET,
               plant.world_frame(), plant.world_frame())[:, idx]
    Jdv  = plant.CalcBiasTranslationalAcceleration(
               ctx, JacobianWrtVariable.kV, ee, FINGER_TIP_OFFSET,
               plant.world_frame(), plant.world_frame()).ravel()
    return M, Cv, tau_g, Jv, Jdv


def _solve_wbqp(self, a_des, f_des, n_hat, mu_c, q, qd, dt, in_contact):
    M, Cv, tau_g, Jv, Jdv = self._dynamics_terms()
    t_hat = np.array([-n_hat[1], n_hat[0], 0.0])
    n3    = np.array([n_hat[0], n_hat[1], 0.0])

    prog = MathematicalProgram()
    vd  = prog.NewContinuousVariables(NQ_ARM, "qdd")
    tau = prog.NewContinuousVariables(NQ_ARM, "tau")
    f   = prog.NewContinuousVariables(3, "f")
    s   = prog.NewContinuousVariables(2, "s")

    # ── Égalité : dynamique.  M·q̈ − τ + Jᵥᵀ·f = τ_g − C·q̇
    Aeq = np.hstack([M, -np.eye(NQ_ARM), Jv.T])
    prog.AddLinearEqualityConstraint(Aeq, tau_g - Cv,
                                     np.concatenate([vd, tau, f]))

    # ── I1 couples (DURE — plus d'écrêtage a posteriori)
    prog.AddBoundingBoxConstraint(-JOINT_TORQUE_LIMITS, JOINT_TORQUE_LIMITS, tau)

    # ── I2 vitesses articulaires
    prog.AddBoundingBoxConstraint((-QD_MAX - qd) / dt, (QD_MAX - qd) / dt, vd)

    # ── I3 positions articulaires (barrière au niveau accélération)
    lo = 2.0 * (Q_MIN - q - qd * dt) / dt**2
    hi = 2.0 * (Q_MAX - q - qd * dt) / dt**2
    prog.AddBoundingBoxConstraint(lo, hi, vd)

    if in_contact:
        # ── I4 cône de frottement (SOUPLE)
        prog.AddLinearConstraint(( t_hat - mu_c * n3) @ f <= s[0])
        prog.AddLinearConstraint((-t_hat - mu_c * n3) @ f <= s[0])
        # ── I5 / I6 maintien et plafond
        prog.AddLinearConstraint(n3 @ f >= F_MIN - s[1])
        prog.AddLinearConstraint(n3 @ f <= F_MAX)
        prog.AddQuadraticErrorCost(W_F * np.eye(3), f_des, f)
    else:
        # ── I7 hors contact
        prog.AddBoundingBoxConstraint(np.zeros(3), np.zeros(3), f)

    prog.AddBoundingBoxConstraint(0.0, np.inf, s)

    # ── Coût
    prog.Add2NormSquaredCost(np.sqrt(W_TASK) * Jv,
                             np.sqrt(W_TASK) * (a_des - Jdv), vd)
    qdd_post = KP_POST * (Q_POSTURE - q) - KD_POST * qd
    prog.AddQuadraticErrorCost(W_POST * np.eye(NQ_ARM), qdd_post, vd)
    prog.AddQuadraticErrorCost(W_REG * np.eye(NQ_ARM), np.zeros(NQ_ARM), tau)
    prog.AddQuadraticCost(RHO * (s @ s), is_convex=True)

    res = Solve(prog)
    if not res.is_success():
        self._n_qp_fail += 1
        return None, None, None
    return (res.GetSolution(tau), res.GetSolution(f), res.GetSolution(s))
```

Branchement dans `_update`, en remplacement de `F_trans = Kv * (v_cmd - v_sph)` puis `send(Jv.T @ F_trans)` :

```python
a_des = K_ACC * (v_cmd - v_sph)               # v_cmd inchangé, boucle externe intacte
f_des = F_NOMINAL * (-self._face_normal)      # force souhaitée le long de la poussée
tau_qp, f_qp, s_qp = self._solve_wbqp(
    a_des, np.array([f_des[0], f_des[1], 0.0]),
    -self._face_normal, self._mu_combined,
    q_arm, qd_arm, self._qp_dt, self._contact)

if tau_qp is not None:
    tau_total[self._arm_vel_idx] = tau_qp      # PAS de np.clip : I1 le garantit
    discrete_state.get_mutable_vector().SetFromVector(tau_total)
    self._qp_log.append({"t": t, "s_cone": s_qp[0], "s_contact": s_qp[1],
                         "fn": float(f_qp[:2] @ -self._face_normal),
                         "ft": float(f_qp[:2] @ np.array([self._face_normal[1],
                                                          -self._face_normal[0]]))})
else:
    send(Jv.T @ (Kv * (v_cmd - v_sph)))        # repli sur l'ancienne loi
```

**Le `np.clip` disparaît.** C'est le geste qui répond à p32 : le couple n'est plus corrigé après coup, il est contraint pendant le calcul. Note-le explicitement dans le mémoire.

---

## Les trois pièges

**1. Cadence.** 19 variables et ~50 contraintes, c'est encore petit pour OSQP (~150 µs), mais construire le `MathematicalProgram` en Python coûte ~1 ms. **À 1 kHz, tu n'y arrives pas.** Décime à 200–250 Hz (`self._qp_dt = 0.004`) et tiens le couple entre deux résolutions. C'est physiquement défendable et ça se dit en une phrase. Si tu veux le 1 kHz, passe par `osqp` directement avec `update()` sur un problème pré-factorisé — la structure creuse ne change pas d'un tick à l'autre, seules les valeurs bougent.

**2. Faisabilité.** I1 + I3 + I5 peuvent se contredire près des butées. D'où la hiérarchie : **dures** = I1, I2, I3 (intégrité matérielle) ; **souples** = I4, I5 (désirables pour la tâche). Logge `s` systématiquement : `s₀ > 0` veut dire « la tâche exigeait de faire glisser le pousseur », donc **le push demandé est mécaniquement infaisable à ce coefficient de frottement**. C'est une information que ton contrôleur actuel ne peut pas produire, et c'est un des meilleurs résultats que ce travail va générer.

**3. `mu_c`.** Coefficient combiné de Drake (moyenne harmonique, ton Eq. 4.21), pas celui du SDF.

---

## Ordre, et l'étude comparative qui en sort

```
Étape 0  Logger p_y, f_n, f_t, a·f_t, τ par articulation → re-run    (0.5 j)
         ↳ répond à p24, p25, p45 d'un coup
         ↳ donne le taux de glissement qui MOTIVE tout le reste
         ↳ ne touche pas au contrôleur : risque nul

Étape 1  QP en vitesse (guide 1)                                      (2 j)
         ↳ filet de sécurité, garanti livrable

Étape 2  QP articulaire (ce guide)                                    (4–5 j)
         ↳ LE livrable attendu
```

Garde l'étape 1 **même après avoir fait l'étape 2** : elle devient une baseline. Ta comparaison finale a alors quatre points, et elle raconte une histoire :

| | Force Push [19] nu | Ton contrôleur actuel | QP tâche | QP articulaire |
|---|---|---|---|---|
| Taux de succès | ? | 28/48 | ? | ? |
| Temps hors cône de frottement | ? | ? | ~0 | ~0 |
| Événements de saturation de couple | ? | ? (fréquents) | ? | **0 par construction** |
| Butées articulaires approchées | ? | non surveillé | non surveillé | **0 par construction** |
| Gains réglés à la main | 2 | 8 | 4 | 4 |

Cette table répond simultanément à p49 (« does not compare against [19] »), p29, p32 et p56. C'est ta section de résultats principale.

---

# Et YOLO / RealSense dans tout ça ?

## L'évaluation franche

Ton script est du vrai code de perception, et c'est précisément ce qui manque au mémoire. Mais il ne peut pas servir tel quel, pour une raison qui n'est pas un détail :

**YOLOv5s pré-entraîné sur COCO ne détectera pas ton cube ni ton cylindre.** COCO a 80 classes — personne, voiture, bouteille, tasse… Un cube rouge uni ou un cylindre lisse sur une table n'en fait pas partie. Tu obtiendras soit aucune détection, soit une classe absurde (« sports ball », « bowl », « cake ») avec une confiance basse et instable. Un détecteur généraliste sur un objet géométrique nu est le mauvais outil : il faudrait le fine-tuner sur tes objets, ce qui veut dire annoter un dataset — plusieurs jours, pour un gain nul par rapport à une segmentation couleur qui marchera mieux sur un objet uni.

**Et le script reproduit exactement le biais que je t'ai signalé.** Tu prends le centre de la boîte englobante, tu lis la profondeur à ce pixel, tu déprojettes :

```python
cx = int((x1+x2)/2);  cy = int((y1+y2)/2)
Z = depth_image[cy, cx]
X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], Z)
```

`Z` est la profondeur de la **surface visible**, pas du centre de l'objet. Pour ton cylindre de rayon 100 mm, ça te place ~100 mm devant le centre réel — quatre fois ton seuil de succès de 25 mm. C'est le même problème en simulation et sur matériel réel, et c'est ce qui rend la correction de biais intéressante : **elle est indépendante du capteur.** Bonne nouvelle pour toi : ça consolide la contribution au lieu de la diluer.

## Cinq bugs à corriger si tu réutilises ce code

1. **Les filtres ne servent à rien.** Tu convertis en numpy *avant* de filtrer, et tu ne reconvertis jamais après. `depth_image` est la version brute.
   ```python
   depth_frame = spatial.process(depth_frame)
   depth_frame = temporal.process(depth_frame)
   depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale   # ← après
   ```
2. **Couleur et profondeur ne sont pas alignées.** Tu prends des pixels de l'image couleur et tu les déprojettes avec `depth_intrinsics`. Sur une D455 les deux flux ont des intrinsèques et une pose différentes. Il faut `align = rs.align(rs.stream.color)` puis `frames = align.process(frames)`. C'est le bug RealSense classique, et il produit une erreur systématique de plusieurs centimètres qui *ressemble* à du bruit.
3. **`np.median` sur toute la boîte** inclut le fond et la table. Sur un objet posé, la médiane peut être la profondeur de la table.
4. **Profondeur invalide non filtrée.** RealSense code « pas de mesure » par 0. Un `Z = 0` déprojette à l'origine de la caméra.
5. **Mineurs :** `object_positions = {}` est réinitialisé à chaque itération, donc ne garde qu'un objet ; `out.release()` référence un `out` jamais défini et se trouve après un `while True:`, donc inatteignable.

## Où ça a réellement sa place

Pas en remplacement du pipeline Drake — ton benchmark de 48 runs a besoin d'une perception **dans la boucle de simulation**, et une D455 ne peut pas y être. Mais il y a une place où ça vaut beaucoup :

**Une campagne de caractérisation sur capteur réel qui calibre ton modèle simulé.** Tu poses ton cube et ton cylindre sur une table, tu mesures avec la D455 l'erreur de localisation réelle en fonction de la distance, de l'angle de vue et de l'occlusion partielle — puis tu **injectes ces statistiques mesurées** dans le modèle de perception de ta simulation.

Ça répond frontalement à ta propre limitation §7.4 (« the camera perception model used here … is itself simulated ») et à la remarque p1 du superviseur, sans exiger que tout le benchmark tourne sur du matériel. Et méthodologiquement c'est fort : au lieu d'inventer σ = 3 mm, tu le mesures.

Pour la segmentation dans cette campagne, oublie YOLO : tes objets sont unis et tu contrôles leur couleur. Un seuillage HSV plus un filtrage par profondeur fera mieux, en dix lignes, sans dataset. Garde YOLO pour ta présentation si tu veux montrer que tu sais le faire — mais ce n'est pas ce qui répond au reproche.
