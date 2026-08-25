# Analyse du feedback — 84 commentaires, 40 pages annotées

## Le verdict en une ligne

Ton superviseur ne te reproche pas de mal avoir fait le travail. Il te dit que **tu n'as pas fait le travail convenu**, et il te le dit dix fois.

> p13 : « **The main agreed constrained-control objective is missing.** Mapping a Cartesian command through the manipulator Jacobian is standard robot control and does not replace the requested constrained-control methodology. Where are robot limits/contact constraints explicitly incorporated into the control law? **This is a central gap in the thesis.** »

> p56 : « **Most importantly, the constrained-control component expected for the thesis is still absent from the proposed methodology.** »

Le « constrained controller » dont tu m'as parlé au troisième message n'était donc pas une idée d'amélioration. C'était **le livrable central que tu devais produire**. Ça change son statut : ce n'est plus la contribution la plus intéressante du plan, c'est la condition de validation du mémoire. Tout le reste passe après.

---

## Répartition des 84 commentaires

| Axe | Nb | Gravité |
|---|---|---|
| A. Contrôle sous contraintes absent | ~12 | **Bloquant** — objectif convenu non livré |
| B. Chapitre 4 incomplet et partiellement faux | ~13 | **Bloquant** — erreurs de modélisation, pas de rédaction |
| C. « Perception » non honorée | ~8 | **Bloquant** — le titre est en cause |
| D. Sur-revendication / claims non soutenus | ~11 | Grave — question d'intégrité scientifique |
| E. Delta vs [19] non articulé | ~7 | Grave — la contribution n'est pas identifiable |
| F. Rigueur expérimentale | ~9 | Grave — corrigeable |
| G. Erreurs ponctuelles et présentation | ~10 | Mineur |

---

## Axe A — Le contrôle sous contraintes (bloquant)

Le fil dominant. Il revient p13, p15, p17, p28, p29 (quatre fois sur la même page), p32, p56.

Le motif est toujours le même : tu **dérives** une contrainte au Chapitre 4, puis il cherche où elle est **imposée** au Chapitre 5, et ne la trouve pas.

> p29 : « Where is this constraint enforced later? I do not see joint-velocity limiting/constraint handling in the control law or results. »

> p29 : « If the controller is allowed to command forces outside this cone and Drake simply resolves the resulting slip, then **this is a system property, not a constraint handled by your controller.** »

> p29 : « This paragraph effectively confirms that **no constrained contact-force optimisation is solved.** This is a major departure from the expected constrained-control part of the thesis and needs to be addressed, not just explained away. »

Et il anticipe précisément la défense que ton code fait aujourd'hui :

> p32 : « **Saturating the resulting torque is not equivalent to designing a controller subject to torque constraints.** Saturation changes the commanded behaviour after the control law is calculated and can affect stability/tracking. »

C'est exactement le `np.clip(tau, -JOINT_TORQUE_LIMITS, JOINT_TORQUE_LIMITS)` de ton `send()`. Il a lu ton architecture correctement.

**Ce qu'il attend, et il le nomme lui-même** (p18, à propos du gap vs [19]) :

> « Please identify what non-trivial manipulator-specific problem you solve here, e.g. **redundancy, joint/workspace limits, torque/velocity constraints or singularity handling.** »

**Action.** Le QP du guide répond à ça, à trois réserves près :

1. Il demande explicitement une **optimisation de force de contact sous contraintes** (p29). Mon QP optimise la vitesse commandée, dont la force dérive de façon affine — c'est équivalent mathématiquement, mais il faut le **présenter** au niveau force pour répondre à la question posée. Formule le QP avec la force comme variable et la vitesse comme sortie, ou démontre explicitement l'équivalence.
2. Ajoute la contrainte C4 (vitesses articulaires) que j'avais marquée optionnelle : il la cite nommément deux fois.
3. **La redondance** (p32) : « The Panda has 7 DoF, yet only the translational task is controlled here. What happens to the remaining redundancy/orientation/null-space dynamics? » Ta tâche est 3D, ton robot en a 7 → 4 dimensions de noyau non traitées. Un QP au niveau articulaire (variable `q̇ ∈ R⁷` au lieu de `v ∈ R²`) traiterait redondance, limites articulaires et cône de frottement **dans le même problème**. C'est plus ambitieux que ce que je proposais, et c'est plus proche de ce qu'il demande.

---

## Axe B — Le Chapitre 4 (bloquant) — ce que j'avais raté

**Je dois être direct : j'ai manqué cet axe entièrement.** Je lisais ton mémoire pour y chercher des trous de contribution, et j'ai pris le Chapitre 4 pour de la théorie correcte mais inutilisée. Ton superviseur y a trouvé des **erreurs de modélisation**. Il a raison sur les points ci-dessous, et c'est plus grave que ce que je te disais, parce qu'une théorie inutilisée se recycle alors qu'une théorie fausse se refait.

**B.1 — Le frottement de la table est absent. C'est l'erreur principale.** Six commentaires (p12, p21, p22 ×3, p23, p28).

> p22 : « **The object is sliding on a table, so where is the table/support friction wrench in this equation?** This is fundamental to planar pushing. `F_o = G f_c` cannot be the complete net wrench if Coulomb friction with the table is also assumed. »

> p22 : « Neglecting acceleration does not by itself provide a force-to-object-velocity relation. **Under quasi-static pushing you still need the support-friction/limit-surface relation.** »

Il a raison, et c'est fondamental. En poussée quasi-statique, la force du pousseur est équilibrée par le frottement de support, et c'est ce **rapport** qui détermine le mouvement — pas la force de contact seule. Ton Eq. (4.2) et ton Eq. (4.16) sont donc incomplètes, et la phrase de ton introduction (« the contact model relating the measured contact force to the object's planar motion ») n'est pas honorée.

Tu cites pourtant la limit surface [6] au §3.2. Il le remarque :

> p16 : « This is highly relevant to the quasi-static pushing model, but **you do not actually use a limit-surface/support-friction relation in Chapter 4.** »

**Bonne nouvelle : cette réparation sert directement l'axe A.** La limit surface fournit la relation force→twist qui permet de *prédire* le mouvement de l'objet. Sans elle, seul un QP à un pas est possible ; avec elle, un MPC devient réalisable. Réparer le Ch. 4 et livrer le contrôle sous contraintes sont le même travail, pas deux travaux.

**B.2 — Erreur de repère dans Eq. (4.4).**

> p23 : « You define `f_n` and `f_t` in the local contact frame but Eq. (4.4) inserts them directly as the first two components of the object wrench. In the general case **you need the appropriate rotation into the frame in which `F_o` is expressed.** »

Correct. Il manque une matrice de rotation contact→objet. À corriger avant d'écrire la contrainte de cône du QP, sinon tu propagerais l'erreur dans le contrôleur.

**B.3 — Erreur de signe entre Eqs. (4.15) et (4.16).**

> p27 : « If `f_c` is the force applied by the robot on the object, the force acting back on the robot is `−f_c`. As written, Eqs. (4.15) and (4.16) appear to use the same force sign on both bodies. »

**B.4 — La simplification τ_o ≈ −p_y·f_n est non justifiée et probablement fausse dans les cas difficiles.**

> p25 : « Why is it acceptable for the implementation to neglect `a·f_t`? Please verify this assumption from the actual simulated forces. **Later you explicitly say that at high friction the tangential term becomes important, which would invalidate this simplification exactly in the difficult cases.** »

C'est une contradiction interne réelle de ton mémoire. Et ça touche mon idée de régulation du yaw par `p_y` : elle repose sur cette même Eq. (4.7). **Il faut mesurer `a·f_t` contre `p_y·f_n` avant de bâtir dessus.** Ça tombe bien, c'est le même log `fx/fy/nx/ny` que celui du taux de glissement.

> p24 : « If `p_y` is claimed to be the dominant factor, please **measure/report `p_y`, `f_n`, `f_t` and the corresponding torque terms in the results.** »

Il te demande littéralement le log que le guide propose d'ajouter. Fais-le en premier.

**B.5 — Le centre de la sphère n'est pas le point de contact.**

> p27 : « The sphere center is not the physical contact point. If the force contains a tangential component, a force applied at the sphere surface can also create a moment about the sphere/tool frame. »

Ça touche ton `FINGER_TIP_OFFSET = [0,0,0]` et ton calcul de jacobienne, qui prennent le centre de la sphère comme point d'application.

**B.6 — Autres :** justification quantitative du quasi-statique (p22, et il note que tes explications de résultats invoquent l'inertie, ce qui est incohérent avec l'hypothèse) ; convention de signe de la gravité (p31) ; propriété du cylindre valide seulement sous contact ponctuel idéal (p26).

---

## Axe C — La perception (bloquant)

Diagnostic identique au mien, formulé dès la page de titre.

> p1 : « The term "Perception" in the title is currently not justified by the methodology. A simulated noisy object-position measurement is not by itself a perception contribution. **Either an actual perception pipeline must be implemented/evaluated or the scope/title needs to represent the actual work more accurately.** »

> p2 : « I would not call this a camera-based perception model. **No image/camera processing or object-localisation method is implemented.** »

> p19 : « This literature is about actual object detection/pose estimation, but none of these methods is implemented in the thesis. **If the thesis only assumes a noisy object-state measurement, I would strongly reduce the claim that perception is part of the contribution.** »

Il te donne explicitement **deux issues** : implémenter, ou réduire le périmètre et le titre. C'est une décision de scope, pas une correction — et c'est la question n°2 à lui poser (voir plus bas).

**Un point supplémentaire que je n'avais pas vu** (p38) : ton modèle de perception du Ch. 5 et tes expériences du §6.6 ne décrivent pas le même système.

> « Here you define a single measurement taken before the push, whereas later you use repeated measurements at 30 Hz with 80 ms latency and probabilistic frame loss. **Please define one complete and consistent measurement model in the methodology before evaluating it.** »

C'est vrai — ta §5.3 décrit une capture unique, ton §6.6 évalue du 30 Hz avec pertes. À corriger même si tu ne fais rien d'autre.

Et sur les capteurs de force, une demande explicite pour ta défense :

> p2 : « maybe **review the existing sensors that can be used for such a case as a preparation for your presentation.** »

---

## Axe D — Sur-revendication

Onze commentaires. C'est l'axe le moins coûteux à corriger et le plus dangereux à ignorer, parce qu'il touche l'intégrité plutôt que la compétence.

**Le plus sérieux** (p55) :

> « **only 28/48 benchmark runs succeed**, so "converges reliably across most of the tested grid" is too strong. »

Ton superviseur a compté. **28/48 = 58 %.** Ce nombre n'apparaît nulle part dans ton mémoire, et ta conclusion dit « converges reliably ». Il faut le donner explicitement et décrire la région de fonctionnement réelle.

> p55 : « The contact-force direction **is not the sole feedback** used by the controller: object position, object yaw and object velocity are also used in the control laws. »

Vérifié dans ton code : `_update` utilise `object_xy`, `theta_o` (via `_theta_o_filt_vec`), et `self._v_obj_filt`. Il a raison, et ta phrase d'ouverture du Ch. 7 est factuellement fausse.

> p2 / p54 : « The statement about robustness to 90% observation loss is not supported by your own results. At 90 % frame loss the mean lateral deviation increases **from 15.8 to 28.3 mm** and only 4/8 runs succeed. »

Ton résumé d'abstract dit « tolerates the loss of up to 90 % of visual observations with no degradation ». Ton propre Tableau 6.5 dit le contraire. La conclusion « no degradation » ne vaut que pour le cas 50 %.

Également : « robustly » (p11) et « efficient »/« robust » (p13) sans définition mesurable ; « proposes » dans l'abstract alors que la loi centrale vient de [19] (p2) ; le seuil de ~30° d'obliquité non établi (p47, seulement 4 obliquités discrètes testées).

---

## Axe E — Le delta vs [19]

Sept commentaires. Il ne sait pas ce qui est de toi.

> p18 : « Since this is the method most closely related to your work, I expect a **much more detailed comparison**: what exactly do you take unchanged from [19], what do you modify, and what is genuinely new? »

> p18 : « [19] uses a UR10 mounted on a Ridgeback, **but the arm joints are fixed and only the mobile base is controlled.** Therefore controlling the Panda arm is a change in the actuation/control interface. However, **this alone is not sufficient as a methodological contribution.** »

**Et il y a une contribution que tu as sans le savoir** (p33, p34) :

> « Your `δ_c` is defined using the sphere position relative to the perceived object position, whereas the Force Push paper defines its lateral term **relative to the desired pushing path/contact point. These are not automatically the same quantity.** … Either derive the equivalence or **present this explicitly as your modification of [19].** »

La légende de ta Figure 5.4 affirme que les deux formulations sont équivalentes. Soit tu le démontres, soit — c'est l'option intéressante — **tu le revendiques comme ta modification**. C'est une contribution gratuite que tu as écrite puis attribuée à quelqu'un d'autre.

Enfin, une erreur de citation (p19) : **[21] est le Contact Particle Filter**, pas un estimateur géométrique par pénétration. À corriger.

---

## Axe F — Rigueur expérimentale

> p54 : « **How many random seeds/repeated trials were used?** Since both are stochastic, **one realization per configuration is not sufficient for a robustness claim.** »

> p49 : « This ablation only shows the effect of disabling two terms within your final implementation. **It does not compare the proposed complete controller against the reference Force Push controller [19].** Without that comparison, the improvement beyond the state of the art is still not quantified. »

Ce sont exactement mes P2.1 et P2.2 (baseline B2). Confirmés par la source qui compte.

> p41 : « The **cube width is 100 mm, whereas the cylinder diameter is 200 mm.** Therefore size and rotational inertia also change between the two objects. **The current experiment does not isolate shape alone.** »

Celui-ci m'avait échappé et il invalide une part notable de ton analyse cube/cylindre. Refais la grille avec des dimensions caractéristiques appariées (même diamètre circonscrit, ou même I_zz).

> p42 : « A factor-of-2–3 dependence on the simulation timestep is significant. Simply selecting 10⁻⁴ s is not sufficient to establish numerical convergence. **Please provide a timestep-convergence check**, especially because the controller directly uses the simulated contact force. »

> p52 : « **This is mathematically incorrect.** `(1/30)/(1−p)` describes an *expected* inter-observation timescale under independent random frame loss; **it is not a worst-case bound.** With probabilistic dropout there is no finite maximum number of consecutive lost frames unless you impose one explicitly. »

Il a raison, c'est une loi géométrique, non bornée. J'aurais dû l'attraper aussi. Ton « roughly 20 mm at worst » du §6.6.1 est à réécrire. **Note que ce problème disparaît de lui-même avec le pipeline RGB-D** : l'occlusion réelle a une durée bornée par la géométrie du bras, pas par un tirage.

Également : p43 (l'explication masse/inertie est fausse — F = μmg fait croître le couple avec la masse, on ne peut pas augmenter I_zz à couple constant) ; p45 (l'échec attribué aux limites de couple n'est pas démontré — montrer les historiques de couple).

---

## Axe G — Ponctuel

p3 hyperliens cassés + listes de tables/figures manquantes · p10 MPC et ICP dans les abréviations mais absents de la méthode · p11 section à illustrer · p30 « planner » → dire *waypoint generation*, et « open-loop » est faux puisqu'il y a un PD cartésien · p30 trajectoire d'approche sans collision « by construction », pas par planification · p32 J^T ne crée pas la compliance · p33 **fréquence de mise à jour du contrôleur jamais donnée** (c'est 1 kHz dans ton code — sans elle tes coefficients de filtre n'ont aucune constante de temps reproductible) · p40 vérifier les unités de l'Eq. 5.6 · p2 décrire le solveur de contact Drake et justifier sa fidélité.

---

## Ce que j'avais raté, et ce que ça dit du plan

Sur les six axes, mon analyse en couvrait quatre (A, C, E, F) et en manquait deux :

- **L'axe B en entier.** Je lisais pour trouver des trous de contribution ; je n'ai pas audité la correction mathématique. C'est l'axe le plus coûteux du feedback.
- **Le confond taille/forme** de la grille de benchmark (p41), et **l'erreur de borne probabiliste** (p52), que j'avais pourtant lus tous les deux.

En revanche, deux choses tiennent : le diagnostic sur la perception est mot pour mot le sien, et le contrôleur sous contraintes — que tu as proposé, pas moi — est bien le cœur du sujet. Le plan et le guide restent valides, mais **leur ordre change** : la réparation du Ch. 4 passe devant, parce que le QP en dépend.

---

## Plan révisé

```
0.  Log de p_y, f_n, f_t, a·f_t  →  re-run                        (0.5 j + machine)
    ↳ répond directement à p24, valide ou invalide Eq. (4.7),
      et donne le taux de glissement qui justifie tout l'axe A

1.  RÉPARER LE CHAPITRE 4                                          (3–4 j)
    · frottement de support / limit surface  (p22, p23, p28)
    · rotation de repère dans Eq. (4.4)      (p23)
    · signes dans Eqs. (4.15)/(4.16)         (p27)
    · justifier ou abandonner τ_o ≈ −p_y·f_n (p25, données du 0.)
    ↳ PRÉREQUIS au QP : cône, repères et signes doivent être justes

2.  CONTRÔLEUR SOUS CONTRAINTES                                    (5–6 j)
    · QP présenté au niveau force            (p29)
    · limites articulaires en vitesse ET couple, pas d'écrêtage (p29, p32)
    · redondance / espace nul du Panda 7-DoF (p32)
    ↳ LE livrable manquant

3.  Corriger tous les claims de l'axe D                            (1 j)
    ↳ 28/48, le 90 % d'occlusion, « sole feedback », « robustly »
    ↳ à faire même si tout le reste échoue

4.  Baseline [19] non modifié + N graines                          (1 j + machine)
    ↳ p49, p54

5.  Pipeline RGB-D                                                 (3 j)
    ↳ selon la réponse à la question 2 ci-dessous

6.  Grille à dimensions appariées + convergence en pas de temps    (machine)
    ↳ p41, p42
```

L'étape 3 est celle qui coûte le moins et rapporte le plus : une journée de rédaction honnête efface onze commentaires et une impression de sur-vente.

---

## Questions à poser à ton superviseur

Six questions, par ordre d'importance. Les trois premières sont bloquantes — n'écris pas de code avant d'avoir les réponses.

**1. Quel niveau de contrôle sous contraintes attendez-vous ?**
Votre commentaire p29 mentionne « constrained contact-force optimisation ». Est-ce que vous attendez (a) un QP à un pas imposant cône de frottement et limites articulaires, (b) le même formulé au niveau articulaire pour traiter aussi la redondance du Panda, ou (c) un MPC sur horizon avec le modèle quasi-statique ? Je peux livrer (a) sûrement, (b) probablement, (c) seulement si le Chapitre 4 est réparé d'abord.

**2. Perception : implémenter ou réduire le périmètre ?**
Vous proposez p1 les deux issues. Je peux implémenter un vrai pipeline RGB-D dans Drake — `RgbdSensor`, segmentation, déprojection, correction du biais de centroïde de surface — pour environ 3 jours. Est-ce que ça vaut mieux que de restreindre le titre et de concentrer tout l'effort sur le contrôle sous contraintes ? Autrement dit : préférez-vous un mémoire qui fait deux choses correctement ou une seule très bien ?

**3. Jusqu'où réparer le Chapitre 4 ?**
Pour le frottement de support, attendez-vous une limit surface complète (approximation ellipsoïdale à la Howe–Cutkosky) ou un équilibre quasi-statique correctement posé avec le torseur de frottement de support explicite ? Le premier permettrait un MPC, le second suffit à rendre le modèle correct.

**4. Le δ_c : défaut ou contribution ?**
Vous notez p33/p34 que mon `δ_c` diffère de celui de [19]. Je ne l'avais pas réalisé. Est-ce que vous voulez que je démontre l'équivalence, ou que je le présente comme une modification assumée de [19] avec une comparaison expérimentale des deux formulations ?

**5. Budget de calcul : grille large ou statistiques solides ?**
Avec N graines, les baselines et le nouveau contrôleur, le coût explose. Préférez-vous une grille réduite avec 10 répétitions et des intervalles de confiance, ou la grille complète avec une seule réalisation ?

**6. Les objectifs du Chapitre 2 sont-ils à réécrire ?**
Vous notez p13 qu'ils décrivent des étapes d'implémentation plutôt qu'un problème de recherche. Voulez-vous valider une reformulation avant que je réécrive les chapitres qui en dépendent ?

---

## Un mot sur le ton du feedback

Ces commentaires sont durs mais ils sont **techniques, précis et constructifs** — il te dit à chaque fois ce qu'il faudrait faire, pas seulement ce qui ne va pas. Un superviseur qui annote 84 fois un mémoire est un superviseur qui compte le faire aboutir. Plusieurs remarques (« Good point, but… », « This is important, but… ») reconnaissent explicitement la valeur de ce que tu as écrit.

Et il faut le dire : le §6.6.2, où tu sépares « pousser précisément » de « s'arrêter précisément », ne reçoit aucune critique. Le cross-check de Coulomb du §6.2 non plus, sinon pour demander d'aller plus loin. Ce qui est bon dans ton travail reste bon.
