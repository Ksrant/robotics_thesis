# Plan de renforcement du mémoire — contributions personnelles

**Principe directeur :** chaque contribution proposée ci-dessous est ancrée sur un des trois objectifs que tu annonces toi-même au Chapitre 2. Le reproche « pas de contribution personnelle » vient presque toujours du même endroit : le mémoire pose des objectifs, puis le simulateur répond à la place du système. C'est le cas ici, deux fois.

---

## Le diagnostic en une phrase

Tes deux canaux de mesure — la position de l'objet et la force de contact — ne sont pas mesurés. Ils sont lus directement dans le moteur physique, puis dégradés artificiellement. Le système que tu valides n'est donc pas le système que tu décris.

| Canal | Ce que le mémoire dit | Ce que le code fait |
|---|---|---|
| Perception | « camera-based perception model », RGB-D évoqué p.18, 52 | `p̂ = p_true + η`, η ~ N(0, σ²I), σ = 3 mm (Eq. 5.18) |
| Force | « contact-force sensing capability at the end-effector » | `ContactResults` de Drake, « treated as the primary source of truth » (Ch. 2) |
| Fallback force | « geometric estimate » | pénétration entre deux géométries connues exactement |
| Occlusion | « contact-induced occlusion » | tirage de Bernoulli, p ∈ {0.5, 0.9}, indépendant de la géométrie |

Le titre du mémoire est *Perception and control*. La moitié « perception » n'est pas honorée : il n'y a pas un pixel dans tout le travail. Tu le reconnais d'ailleurs à demi-mot en 7.4 (« the camera perception model used here … is itself simulated ») et tu le renvoies en *future work* (7.5). **Ce qui est actuellement en future work est exactement ce qui manque pour avoir une contribution.** La bonne nouvelle : Drake fournit tout nativement, et tu as dit pouvoir relancer des simulations.

---

## Priorité 0 — Crédibilité structurelle

### P0.1 — Pipeline RGB-D réel dans la simulation

**Objectif adressé :** le titre, la section 5.3, tout le chapitre 6.6.

**Ce qui remplace quoi.** `RgbdSensor` de Drake, monté en position fixe sur la scène, produit trois images à chaque tick : RGB (`ImageRgba8U`), profondeur (`ImageDepth32F`, en mètres) et labels (`ImageLabel16I`). Le pipeline devient :

1. **Segmentation** de l'objet dans l'image RGB par seuillage couleur en espace HSV (l'objet reçoit une couleur distinctive dans le SDF/URDF). C'est une opération de vision réelle, pas un oracle.
2. **Masque → pixels de profondeur.** Pour chaque pixel `(u, v)` du masque, on lit `d = depth[v, u]`. On rejette `d = +inf` (au-delà de la portée) et `d = 0` (trop proche).
3. **Déprojection** avec les intrinsèques réelles de `CameraInfo` :

   `p_C = [ (u − c_x)·d / f_x , (v − c_y)·d / f_y , d ]`

4. **Transformation** `p_W = X_WC · p_C`.
5. **Estimation de position** à partir du nuage de points obtenu.

**Le point non trivial, et c'est là qu'est ta contribution.** Le centroïde du nuage déprojeté n'est **pas** le centre de masse : une caméra ne voit qu'une face de l'objet, donc le centroïde de la surface visible est biaisé vers la caméra d'environ *R* pour le cylindre, *a* pour le cube. Ce biais est systématique, de l'ordre de 50 mm sur tes objets — soit **deux fois ton seuil de succès de 25 mm**. L'ignorer fait échouer tous les runs ; le corriger est une vraie contribution de perception :

- **Cylindre :** ajustement du rayon connu *R* sur le nuage projeté au sol (fit de cercle par moindres carrés sur l'arc visible), ou correction analytique le long de l'axe caméra→objet.
- **Cube :** ajustement de la face visible (plan dominant par RANSAC), puis recul de *a* le long de la normale de ce plan. Bonus : **l'orientation θ_o du cube devient observable** depuis la normale du plan — ce qui te donne une mesure du yaw que tu n'as pas aujourd'hui, et qui alimente directement P1.1.

**Ce que ça débloque gratuitement :**

- **L'occlusion devient émergente.** Le bras occulte réellement l'objet pendant le push. Tu remplaces `p = 0.9` (arbitraire) par une courbe « nombre de pixels objet visibles vs temps », corrélée temporellement et dépendante de la géométrie — ce qu'une occlusion de Bernoulli ne capture jamais. La remarque de ta section 6.6.1 (« a real RGB-D pipeline, partially occluded by the manipulator, operates in the intermittent regime ») devient une mesure au lieu d'une supposition.
- **Une section d'évaluation de la perception**, que le mémoire n'a pas du tout : IoU du masque de segmentation contre l'image de labels (utilisée comme vérité terrain, jamais comme entrée du contrôleur), erreur de position vs distance caméra, vs obliquité, vs taux d'occlusion.
- **Ta conclusion principale se reteste dans des conditions honnêtes.** « The controller does not require continuous tracking to push accurately; it requires it to stop accurately » est ton meilleur résultat. Aujourd'hui il est démontré sur un modèle de bruit que tu as écrit toi-même — un jury dira que tu as prouvé une propriété de ton générateur de nombres aléatoires. Sous occlusion réelle, ce même résultat devient solide. Et s'il ne tient plus, c'est un résultat également publiable, à condition de l'expliquer.

**Effort :** ~2–3 jours (setup `RenderEngineVtk` + pipeline + validation).

---

### P0.2 — Estimation de la force de contact par les couples articulaires

**Objectif adressé :** Objectif 3 (« Regulate the command and confirming contact »).

Ton Chapitre 2 annonce que la confirmation de contact repose sur « two independent sources ». Les deux sont de la vérité terrain : le solveur de contact de Drake, et la pénétration géométrique entre deux corps dont tu connais exactement les poses. Ce ne sont pas deux sources indépendantes, ce sont deux vues du même oracle. L'objectif 3 n'est donc pas réellement atteint.

**Remplacement.** Tu cites déjà [20] dans l'état de l'art (§3.4.3, estimation par les couples articulaires) sans jamais l'utiliser. Fais-le :

1. Récupérer τ_ext via `get_generalized_contact_forces_output_port()` (l'analogue simulé de `tau_ext_hat_filtered` que le vrai Franka publie).
2. Ajouter un modèle de capteur : bruit gaussien sur chaque couple, quantification, filtrage passe-bas — un couple articulaire réel n'est ni exact ni instantané.
3. Inverser : `f̂ = (J_v^T)^+ · τ_ext`, pseudo-inverse de Moore-Penrose.

**Pourquoi c'est une contribution et pas juste du travail en plus :** cette inversion est **mal posée**. `J_v^T` est 7×3, son noyau n'est pas trivial, donc une partie de τ_ext n'est pas attribuable au contact. L'erreur d'estimation dépend de la configuration articulaire — elle explose près des singularités. Tu obtiens donc :

- une **figure de validation** `‖f̂ − f_Drake‖` au cours du push, avec le conditionnement de la jacobienne en abscisse ;
- une **caractérisation** : dans quelles régions du workspace ton canal de force est fiable ;
- une **conséquence contrôle** : ta loi Eq. (5.6) utilise la *direction* de la force, donc c'est l'erreur angulaire sur f̂ qui compte, pas la norme. C'est une analyse fine que personne ne peut te reprocher d'avoir sous-traitée au simulateur.

**Effort :** ~1–2 jours.

---

## Priorité 1 — La contribution algorithmique : contrôleur sous contraintes

### P1.0 — Reformuler le push comme un QP sous contraintes

**Objectif adressé :** Objectifs 2 et 3, et surtout — il rend le Chapitre 4 porteur.

**Le problème que ça règle, et il est plus grave que celui de la perception.** Ton Chapitre 4 dérive un modèle mécanique complet : le cône de frottement (Eq. 4.20, β = arctan(μ_p)), les contraintes de contact et de frottement (§4.2.2), les contraintes robot et les limites de couple (§4.2.1), le rayon de giration (Eq. 4.9, 4.13). Ton Chapitre 5 n'en utilise **aucun**. Le contrôleur est une pile de termes correctifs empiriques — correction par la force, centrage latéral, filtrage de direction, saturation d'admittance, plafond dur γ_v, amortissement de survitesse, filet stall/regression — dont chacun a été ajouté après avoir observé un mode d'échec, avec des gains « tuned by hand … on a single nominal configuration » (§5.4, tes mots).

Un jury lira ça comme : *un chapitre de théorie décoratif, puis un empilement d'heuristiques*. Et il aura raison, parce que rien dans le Ch. 5 ne dépend du Ch. 4.

Un contrôleur sous contraintes inverse exactement ce rapport. Les contraintes que tu as dérivées deviennent les contraintes du problème d'optimisation, et le Chapitre 4 devient la fondation du Chapitre 5 au lieu d'en être le décor. **C'est la seule modification du plan qui reconfigure la structure entière du mémoire plutôt que d'y ajouter une brique.**

**Formulation (QP à un pas, niveau vitesse).** Ta boucle interne est déjà une impédance en vitesse de gain K_v = 600 N·s/m, donc la force commandée est **affine** en la vitesse commandée :

`f(v) = K_v · (v − v_ee)`

C'est ce qui rend tout le problème quadratique. Avec n̂ la normale de contact et t̂ la tangente, `f_n(v) = n̂ᵀK_v(v − v_ee)` et `f_t(v) = t̂ᵀK_v(v − v_ee)` sont linéaires en v.

Variables de décision : `v ∈ R²` (vitesse de poussée planaire) + variables d'écart `s ≥ 0`.

**Coût :**

```
min   ‖v − V(d)·d̂_goal‖²_Q          suivi de la direction de poussée
 v,s + w_θ ‖t̂ᵀ(v − v_obj) − ṗ_y*‖²  régulation du yaw  (← absorbe P1.1)
     + λ ‖v − v_prev‖²               lissage  (← remplace le filtre de direction D_d̂)
     + ρ ‖s‖²                        pénalité d'écart
```

**Contraintes :**

| # | Contrainte | Expression | Remplace |
|---|---|---|---|
| C1 | Cône de frottement / non-glissement | `\|f_t(v)\| ≤ μ_p·f_n(v) + s₁` | **rien — Eq. 4.20 n'est aujourd'hui jamais imposée** |
| C2 | Maintien du contact | `f_n(v) ≥ F_min − s₂` | timeout T_nc + reposition |
| C3 | Plafond de force | `f_n(v) ≤ F_max` | saturation d'admittance |
| C4 | Vitesses articulaires | `q̇ = J⁺v`, `\|q̇_i\| ≤ q̇_max,i` | **rien — aucune conscience des limites articulaires** |
| C5 | Couples articulaires | `τ = Jᵀf(v) + g(q)`, `\|τ_i\| ≤ τ_max,i` | clipping (à vérifier dans ton code) |
| C6 | Vitesse / freinage | `‖v‖ ≤ V(d)` | profil de freinage, γ_v, V_damp |

Toutes linéaires en v, sauf C6 (norme) qu'on approxime par un octogone — 8 inégalités linéaires, erreur < 4 %. **Résultat : 2 variables, ~20 contraintes linéaires, coût quadratique.** Un QP dense trivial, résolu en quelques dizaines de microsecondes par OSQP. Aucun problème de temps réel à ton pas de commande.

**C1 est la contrainte importante, et elle a un nom dans ta bibliographie.** Maintenir la force de contact dans le cône de frottement, c'est exactement la condition de *stable pushing* de Lynch et Mason [5] — que tu cites en §3.2 (« link the curvature of the object boundary at the contact point to the feasibility of maintaining a stable push ») et que tu n'utilises jamais. Aujourd'hui ton contrôleur ne sait pas si le pousseur glisse sur l'objet ; il l'espère. Avec C1, le non-glissement devient garanti par construction quand le QP est faisable, et **son infaisabilité devient un signal exploitable** : elle te dit que la tâche demandée est mécaniquement impossible avec ce coefficient de frottement, ce qui est une information que ton contrôleur actuel ne peut pas produire.

**Un diagnostic entièrement nouveau, que le mémoire actuel ne peut pas fournir.** Mesure, sur tes 48 runs existants, la fraction du temps de push pendant laquelle la force de contact sort du cône de frottement. Je parierais que c'est non négligeable à μ_p = 0.3. Cette courbe « taux de glissement vs μ_p vs obliquité » est une figure que seul ce travail peut produire, et elle explique mécaniquement les échecs que tu attribues aujourd'hui à « the parasitic torque of Eq. (4.6) ».

**Argument quantitatif à faire valoir en défense.** Six gains empiriques du Tableau 5.1 — K_F, K_C, K_lat, V_damp, γ_v, D_d̂ — disparaissent, remplacés par quatre poids de coût (Q, w_θ, λ, ρ) **et par des paramètres physiques non ajustables** : μ_p se mesure, τ_max et q̇_max sont dans la datasheet du Panda, F_min est un seuil de détection. Tu passes d'un contrôleur réglé à un contrôleur spécifié.

**Faisabilité et priorités entre contraintes.** Le QP peut devenir infaisable (cône + maintien de contact + limites de couple en conflit). C'est à traiter explicitement, et c'est du bon matériau de rédaction : hiérarchiser les contraintes en **dures** (C4, C5 — sécurité matérielle, jamais relâchées) et **souples** (C1, C2 — désirables pour la tâche, relâchées par variables d'écart avec pénalité forte). Justifier cette hiérarchie est un raisonnement d'ingénieur que le jury appréciera.

**Implémentation Drake.** `pydrake.solvers.MathematicalProgram` + OSQP, tous deux fournis. Pour référence sur les contraintes articulaires, `DoDifferentialInverseKinematics` est déjà un contrôleur de vitesse sous contraintes basé QP dans Drake — mais il n'a pas de cône de frottement, donc écris ton propre QP : c'est précisément le cône qui fait la contribution.

**Ce que ça donne comme étude comparative.** C'est l'occasion d'une vraie comparaison à trois, sur la même grille de benchmark :

- **B2** — Heins et al. [19] non modifié
- **B3** — ton contrôleur réactif actuel, avec sa pile de correctifs
- **B4** — le contrôleur sous contraintes

Avec, en métriques : taux de succès, distance finale, **fraction de temps hors du cône de frottement**, violations de couple, et nombre de déclenchements du filet stall/regression.

**Effort :** ~4–5 jours. Le QP lui-même est une demi-journée ; le reste est le débogage de faisabilité et le tuning des poids.

---

### P1.1 — Régulation du yaw par `p_y` (absorbée dans le coût du QP ci-dessus)

> **Note :** cette idée était initialement proposée comme un terme séparé. Le QP de P1.0 l'absorbe proprement via son terme de coût `w_θ`. Je la garde détaillée ici parce que la dérivation reste nécessaire pour écrire ce terme — mais ce n'est plus une contribution distincte, c'est un objectif du contrôleur sous contraintes.

#### Placement de contact piloté par le couple (`p_y` comme entrée de commande)

**Objectif adressé :** Objectif 1 (« Determine where in the geometry of the object to push »), et il ferme le trou théorie→implémentation.

C'est, à mon avis, la modification qui change le plus la perception de ton travail. Trois faits, déjà tous dans ton mémoire, qui ne se sont jamais rencontrés :

1. Tu dérives τ_o ≈ −p_y·f_n (Eq. 4.7) : le décalage latéral sur la face **génère** le couple parasite.
2. Tu dérives c² = ⅔a² (Eq. 4.9) et c² = R²/2 (Eq. 4.13) sur deux pages… puis tu écris deux fois que ces valeurs sont « computed at initialisation and logged for diagnostics », qu'elles « do not feed any closed-loop term ». C'est de la théorie morte dans le document.
3. Ton mode d'échec dominant est le cube à forte obliquité, causé par l'axe de face gelé au planning (§6.4.1, §6.6.2, cube à 247.9 mm).

Ces trois faits disent la même chose. `p_y` n'est pas une perturbation subie : **c'est une entrée de commande que tu n'utilises pas.** Aujourd'hui ton terme de *lateral centering* pousse p_y → 0, c'est-à-dire qu'il annule ton seul actionneur en rotation.

**La proposition :** au lieu de réguler p_y vers zéro, réguler p_y vers la valeur qui produit le couple nécessaire à corriger l'erreur de yaw de l'objet.

```
θ_err  = θ_désiré − θ̂_o                    (θ̂_o mesuré par le fit de plan de P0.1)
τ_cmd  = K_θ·θ_err − K_ω·ω̂_o               (PD sur le yaw)
p_y*   = −τ_cmd / max(f̂_n, f_min)          (inversion de Eq. 4.7)
p_y*   ← clip(p_y*, −a·λ, +a·λ)             (λ ≈ 0.6, on reste dans la face)
```

et le gain K_θ se normalise par l'inertie réelle : `K_θ = k·m·c²`. **C'est là que c² entre enfin dans la boucle fermée**, et l'objection « votre théorie ne sert à rien » disparaît.

**Ce que ça te donne :**

- Le *lateral centering* actuel devient le cas particulier τ_cmd = 0 de ta nouvelle loi. Ce n'est pas un remplacement, c'est une **généralisation** — argument très fort en défense.
- Le mode d'échec dominant est attaqué à sa racine mécanique, pas contourné par un filet de sécurité.
- Une ablation naturelle et propre : centrage latéral classique vs placement piloté par le couple, sur les configurations à forte obliquité (positions A et D).
- Une explication du contraste cube/cylindre qui devient prédictive : le cylindre a τ_o = R·f_t, donc **pas** de levier normal, donc cette loi ne s'applique pas à lui — ce qui prédit que le gain sera concentré sur le cube. Si les données le confirment, c'est une belle boucle théorie→prédiction→mesure.

**Risque à surveiller :** couplage entre la boucle de yaw et la boucle de position (bouger p_y déplace aussi le point d'application de la poussée). Prévoir une séparation d'échelles de temps — boucle de yaw plus lente d'un facteur ~5 — et le documenter comme choix de conception.

**Effort :** ~3–4 jours avec le tuning.

---

### P1.2 — Extension MPC (optionnelle, si le temps le permet)

Le QP à un pas règle les contraintes mais reste **myope**. Or ton propre Tableau 3.1 identifie « little anticipation » comme *la* limite des méthodes réactives, et ta section 6.4.2 décrit un mode de divergence causé par le fait que « the velocity profile reacts only to the magnitude of the remaining distance and not to the sign of progress ».

C'est littéralement un défaut d'anticipation. Le filet stall/regression est un pansement sur l'absence de prédiction.

Étendre le QP à un horizon court (N = 10–20 pas) avec ton modèle quasi-statique du Ch. 4 comme dynamique de prédiction donne une hypothèse testable et élégante :

> **Sous MPC, le filet stall/regression devient inutile.**

Tu peux le vérifier directement : rejoue la configuration divergente de la Figure 6.7 (cube 0.5 kg, μ = 0.7, position A), filet désactivé, sous MPC. Si elle converge, tu as remplacé une heuristique par une propriété structurelle — et tu te raccroches à Hogan & Rodriguez [9], déjà dans ton état de l'art §3.3.3. L'horizon fait aussi de `p_y` un **état prédit** plutôt qu'une quantité mesurée instantanément, ce qui rend la régulation du yaw nettement plus propre.

**À ne faire que si P0 et P1.0 sont terminés et validés.** Un QP à un pas qui marche vaut infiniment mieux qu'un MPC à moitié débogué.

---

## Priorité 2 — Blindage méthodologique

### P2.1 — Répétitions et intervalles de confiance

**Ceci n'est plus optionnel une fois P0.1 et P0.2 faits.** Aujourd'hui la simulation est déterministe, donc 1 run par configuration se défend. Dès qu'il y a du bruit capteur et de la segmentation, chaque run est un tirage : 48 runs uniques deviennent statistiquement indéfendables.

- N ≥ 10 graines par configuration (bruit caméra, bruit couple, jitter ±5 mm / ±5° sur la pose initiale).
- Rapporter moyenne ± IC 95 %, et le taux de succès avec un intervalle de Wilson (binomial, petit N).
- Ce point règle aussi une gêne réelle du Ch. 6 : ta remarque « mean and median must be read together here » (§6.6.2), où deux runs dominent une moyenne sur huit, disparaît dès qu'il y a une vraie distribution.

**Effort :** ~1 jour de code, du temps machine ensuite.

### P2.2 — Baselines explicites

Tu n'as aucun point de comparaison. L'ablation compare ton contrôleur à lui-même amputé, ce qui ne dit pas s'il vaut mieux que trivial. Deux baselines suffisent :

- **B1 — push en position pure, boucle ouverte :** l'end-effector suit la ligne droite start→goal sans retour de force. Isole la valeur du canal de force.
- **B2 — Heins et al. [19] non modifié :** loi Eq. (5.6) seule, sans centrage latéral, sans filtrage de direction, sans filet stall/regression. Isole ta propre adaptation.

B2 est important : ta contribution n°1 est formulée comme « an adaptation of [19] ». Sans B2, tu ne quantifies jamais ce que ton adaptation apporte, et c'est exactement la question qu'on te posera.

**Effort :** ~1 jour (B1 est presque déjà là dans ta FSM).

### P2.3 — Corriger le critère d'arrêt (quasi gratuit)

Ta section 6.6.4 identifie le problème, propose deux corrections — élargir `d_done` de l'erreur de perception attendue, ou exiger la condition sur plusieurs trames consécutives — et n'en implémente ni ne teste aucune. Un jury verra une conclusion laissée à mi-chemin. Implémente les deux, teste-les, mets le résultat dans un tableau. C'est une demi-journée pour transformer une observation en résultat.

---

## Priorité 3 — Finitions

- **Test du filtre de direction dans son régime.** Tu écris honnêtement que l'ablation « does not strongly excite the near-goal reversal regime » (§6.5). Construis une configuration qui l'excite délibérément (goal proche, obliquité forte) et conclus pour de bon.
- **Renuméroter les objectifs dans les résultats.** Ajoute en tête de chaque section du Ch. 6 la mention explicite de l'objectif traité. C'est cosmétique mais ça répond littéralement à « tu ne réponds pas assez à tes problèmes ».
- **Coquille p.56 :** « is itself simulatedk ».

---

## Ordre d'exécution recommandé

```
P0.1 RGB-D  ──┬──> P1.0 contrôleur QP sous contraintes ──> P1.2 MPC (optionnel)
              │         (θ̂_o vient du fit de plan,
P0.2 Force  ──┘          f̂ vient de l'estimateur de couple)
                                    │
                                    └──> P2.2 baselines B2/B3/B4
P2.1 répétitions ────────────────────────────> P2.3 critère d'arrêt
```

**Chemin critique : P0.1 → P1.0.** Le QP a besoin de deux entrées que tu n'as pas encore honnêtement : la normale de contact n̂ et la force f̂ (pour C1, C2, C3), et le yaw θ̂_o (pour le terme w_θ). Faire le QP avant la perception et l'estimation de force reviendrait à construire un contrôleur rigoureux alimenté par des oracles — le même reproche qu'aujourd'hui, en plus sophistiqué.

**Une exception, à faire tout de suite.** La mesure du taux de glissement (fraction du temps hors du cône de frottement) peut se calculer **dès maintenant, en post-traitement de tes 48 runs existants**, sans rien réimplémenter : tu as déjà les `ContactResults` de Drake, donc f_n et f_t. Si le taux est élevé, tu as immédiatement la justification empirique de tout P1.0 — et une figure à montrer avant même d'avoir écrit une ligne de QP. C'est une demi-journée pour dérisquer cinq jours de travail. Commence littéralement par ça.

---

## Comment se réécrit la section « Contributions » (7.3)

Aujourd'hui, tes quatre contributions se lisent comme : une adaptation, un assemblage de termes correctifs, un modèle de bruit, et un benchmark. Après ce plan :

1. **Un pipeline de perception RGB-D complet** — segmentation, déprojection, et une **correction du biais de centroïde de surface par ajustement de primitive connue**, sans laquelle une caméra unique produit une erreur systématique de l'ordre de deux fois le seuil de succès de la tâche.
2. **Une estimation de la force de contact par les couples articulaires**, avec caractérisation de l'erreur d'estimation en fonction du conditionnement de la jacobienne, montrant que la loi réactive est sensible à l'erreur *angulaire* et non à l'erreur de norme.
3. **Un contrôleur de poussée sous contraintes**, formulé comme un QP résolu à chaque pas, qui impose explicitement le cône de frottement de l'Eq. (4.20) — c'est-à-dire la condition de *stable pushing* de Lynch et Mason — ainsi que les limites articulaires et de couple du §4.2. Il remplace six gains correctifs réglés à la main par quatre poids de coût et des paramètres physiquement mesurables, traite le décalage latéral p_y comme une entrée de commande plutôt que comme une perturbation, et fait du modèle mécanique du Chapitre 4 la fondation effective de la loi de commande au lieu d'un préliminaire décoratif.
4. **Une caractérisation de l'occlusion réelle induite par le manipulateur**, montrant qu'elle est structurée et corrélée temporellement, là où un modèle de perte de trame indépendante la sous-estime.
5. **Le benchmark systématique**, désormais avec baselines et intervalles de confiance.

La différence est nette : on passe de « j'ai adapté un contrôleur existant et je l'ai testé » à « j'ai construit une chaîne perception→estimation→contrôle complète, j'ai identifié le mécanisme de son mode d'échec dominant, et je l'ai corrigé en utilisant le modèle mécanique que j'avais dérivé ».

---

## Ce qui est déjà bon et qu'il faut garder

Pour être juste, plusieurs choses tiennent très bien et il ne faut pas les diluer :

- La séparation entre « pousser précisément » et « s'arrêter précisément » (§6.6.2) est un vrai résultat, fin et bien argumenté.
- L'analyse comparée cube/cylindre (τ_o = −p_y·f_n vs τ_o = R·f_t) est mécaniquement juste et explique les données.
- La découverte du mode stall/regression et sa correction sont un travail d'ingénieur honnête.
- Le cross-check des forces de contact contre la prédiction de Coulomb (§6.2), qui a révélé que le pas de temps par défaut surestimait les forces d'un facteur 2–3, est exactement le genre de rigueur qu'un jury remarque.
- Les moments où tu qualifies tes propres conclusions (« this is offered here as a plausible mechanical explanation rather than a fully isolated cause ») sont à conserver absolument. C'est de l'honnêteté scientifique, pas de la faiblesse.
