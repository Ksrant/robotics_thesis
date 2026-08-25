# Comment choisir le contrôleur sous contraintes

> Ce document est écrit pour devenir une **section de ton Chapitre 5**. La justification du choix de méthode est un livrable, pas une note de travail : ton mémoire actuel ne justifie aucun choix de conception, et c'est une des raisons pour lesquelles il se lit comme une accumulation d'heuristiques.

---

## 1. Ce que « contrôle sous contraintes » recouvre réellement

Il existe six familles. Elles ne résolvent pas le même problème, et la plupart des gens les confondent.

| Famille | Principe | Garantit quoi | Coût |
|---|---|---|---|
| **Saturation / écrêtage** | On calcule la loi, puis on tronque | Rien. Modifie la commande *après* coup et change la **direction** du vecteur, pas seulement sa norme | ~0 |
| **Reference governor** | On modifie la *référence* pour que la commande reste admissible | Satisfaction des contraintes, si l'ensemble invariant est calculable | Faible à moyen |
| **Projection dans l'espace nul** (task-priority) | Les tâches secondaires vivent dans le noyau de la principale | Bonne gestion des **égalités** hiérarchisées | Faible |
| **CBF-QP** (barrières) | Chaque contrainte devient un ensemble invariant vers l'avant | Invariance formelle : si on part admissible, on le reste | Faible |
| **QP ponctuel** (TSID / whole-body) | À chaque tick, minimiser l'erreur de tâche **sous** les contraintes | Satisfaction à chaque instant, optimalité instantanée | Moyen |
| **MPC** | Optimiser sur un horizon avec un modèle de prédiction | Satisfaction *anticipée* — évite d'entrer dans un état sans issue | Élevé |

La première ligne est ce que fait ton code aujourd'hui (`np.clip` dans `send()`), et c'est précisément ce que ton superviseur refuse :

> p32 : « Saturating the resulting torque **is not equivalent to** designing a controller subject to torque constraints. »

---

## 2. Les six critères qui discriminent

C'est la partie qui manquait à ma réponse précédente. On ne choisit pas une méthode parce qu'elle est à la mode, on la choisit parce que la **structure du problème** l'impose.

### Critère 1 — De quelle nature sont tes contraintes ?

| Contrainte | Type | Conséquence |
|---|---|---|
| Couples `\|τ\| ≤ τ_max` | sur l'**entrée** | Traitable par n'importe quelle méthode |
| Vitesses articulaires | sur l'**état**, degré relatif 1 | Il faut prédire d'un pas |
| Positions articulaires | sur l'**état**, degré relatif 2 | Il faut une forme barrière ou un horizon |
| **Cône de frottement** | sur une **variable algébrique** (la force de contact) | Il faut la force comme grandeur manipulable |

Tes contraintes sont **hétérogènes**. C'est le premier fait décisif : une méthode qui ne traite qu'un seul type ne suffit pas. Ça élimine la saturation et la projection dans l'espace nul seule.

### Critère 2 — La force de contact doit-elle être une variable de décision ?

> p29 : « This paragraph effectively confirms that **no constrained contact-force optimisation is solved.** »

Ton superviseur emploie les mots « contact-force optimisation ». Le cône de frottement est une contrainte **sur `f`**. Deux façons de faire :

- **f implicite** — tu l'exprimes comme fonction affine de la commande (c'est mon guide 1 : `f = Kv·(v − v_ee)`). Mathématiquement correct, mais `f` n'apparaît pas dans la formulation.
- **f explicite** — `f` est une variable du problème, contrainte dans son cône, liée au couple par l'équation de la dynamique.

Le second répond à la question posée telle qu'elle est posée. **Ce critère seul impose une méthode fondée sur l'optimisation.**

### Critère 3 — As-tu un modèle de prédiction ? ← *le critère décisif*

Pour faire du MPC, il faut savoir prédire où va l'objet quand on le pousse. Or :

> p22 : « Neglecting acceleration does not by itself provide a force-to-object-velocity relation. **Under quasi-static pushing you still need the support-friction/limit-surface relation.** »

**Tu n'as pas ce modèle.** Le frottement de support manque à ton Chapitre 4. Sans lui, `f → mouvement de l'objet` n'existe pas dans ton travail, donc **le MPC est structurellement impossible aujourd'hui**, quelle que soit ta puissance de calcul.

C'est le critère qui tranche, et il est contingent : il changera si tu répares le Ch. 4. J'y reviens en §4.

### Critère 4 — Ton budget de calcul

Ta boucle tourne à 1 kHz, en Python. Un QP ponctuel de 19 variables se résout en ~150 µs. Un MPC sur 20 pas avec un modèle non linéaire, non. Ça ne disqualifie pas le MPC en principe — on décime — mais ça pèse.

### Critère 5 — As-tu de la redondance à résoudre ?

> p32 : « The Panda has 7 DoF, yet only the translational task is controlled here. **What happens to the remaining redundancy/null-space dynamics?** »

Tâche 3D, robot 7 DoF → 4 dimensions non spécifiées. Aujourd'hui elles sont laissées à la dynamique du système sans que tu en dises rien. Il te faut une méthode qui les résolve **explicitement**. La projection dans l'espace nul le fait ; un QP le fait aussi, via un coût secondaire, et en gérant les inégalités en prime.

### Critère 6 — Quelle est la géométrie de tes contraintes ?

Celui-ci décide de la *classe* du problème d'optimisation, et il est spécifique à ton cas :

- En **3D**, le cône de frottement `‖f_t‖ ≤ μ f_n` est un cône de Lorentz → problème conique du second ordre (SOCP), ou approximation pyramidale.
- En **2D**, `|f_t| ≤ μ f_n` avec `f_t` scalaire est **exactement deux inégalités linéaires**.

Ton problème est planaire (Hypothèse 2 du §4.1.2). **Ton cône de frottement est donc exactement linéaire — sans aucune approximation.** Combiné à un coût quadratique et à une dynamique linéaire en `(q̈, τ, f)`, la classe du problème est un **QP**, pas un SOCP ni un NLP.

---

## 3. Application à ton cas

| Critère | Constat | Élimine |
|---|---|---|
| 1. Contraintes hétérogènes (entrée + état + algébrique) | oui | saturation, espace nul seul |
| 2. Force de contact comme variable explicite | demandé | tout ce qui n'est pas optimisation |
| 3. Modèle de prédiction de l'objet | **absent** | **MPC** |
| 4. 1 kHz, Python | serré | horizons longs |
| 5. Redondance 7 DoF / tâche 3D | à résoudre | méthodes sans tâche secondaire |
| 6. Problème planaire | cône exactement linéaire | SOCP, NLP (inutiles ici) |

**Ce qui reste : une optimisation ponctuelle, à coût quadratique et contraintes linéaires, avec la force de contact et les couples comme variables.** C'est-à-dire un QP au niveau articulaire.

Ce n'est pas moi qui ai choisi le QP. C'est ce qui reste quand on applique les six critères.

### Les objections à traiter d'avance

**« Pourquoi pas simplement mettre à l'échelle la commande pour rester dans le cône ? »** Parce que la mise à l'échelle uniforme n'est pas une projection : avec deux contraintes actives simultanément, elle ne trouve pas le point admissible le plus proche, et elle ne dit rien de la redondance. C'est aussi, mot pour mot, la saturation que le superviseur rejette.

**« Pourquoi pas la projection dans l'espace nul ? »** Elle gère très bien la hiérarchie de tâches en **égalité**, mais les **inégalités** (butées, cône) s'y traitent par activation/désactivation, ce qui crée du chattering aux frontières et ne donne aucune garantie quand plusieurs contraintes sont actives. Le QP gère les deux dans le même problème.

**« Pourquoi pas les CBF ? »** Bonne question, et la réponse est nuancée : **tu en fais déjà.** Ma contrainte de butée articulaire `q̈ ≤ 2(q_max − q − q̇·dt)/dt²` est une barrière à temps discret. Tu peux tout à fait présenter ta contrainte I3 dans le cadre CBF — ça te donne un vocabulaire théorique et des citations, gratuitement. Mais les CBF ne remplacent pas le QP : la formulation standard **est** un QP (« CBF-QP »). Ce sont deux niveaux de description, pas deux alternatives.

**« Pourquoi pas un QP hiérarchique (HQP) ? »** C'est la version rigoureuse de ce que j'approxime par des poids : priorités strictement lexicographiques au lieu de `w_task ≫ w_post`. Plus propre théoriquement, plus lourd en implémentation, et pour ton cas le gain est marginal — tu as deux niveaux de priorité, pas cinq. Mentionne-le comme alternative écartée et justifie l'écart par un argument de complexité. Un jury apprécie qu'on connaisse l'option qu'on n'a pas prise.

---

## 4. Le point le plus important : ce choix n'est pas définitif

Le critère 3 est le seul qui élimine le MPC, et il est **réparable**. Si tu ajoutes le frottement de support / la limit surface au Ch. 4 — ce que le superviseur te demande de toute façon, p16 et p22 — alors tu obtiens la relation `force → twist de l'objet`, donc un modèle de prédiction, donc le MPC devient possible.

Et là, une deuxième chose se débloque. Ton propre Tableau 3.1 classe les méthodes réactives avec pour limite « little anticipation ». Ton §6.4.2 décrit un mode de divergence où « the velocity profile reacts only to the magnitude of the remaining distance and not to the sign of progress ». C'est un défaut d'anticipation, et ton filet stall/regression est le pansement posé dessus.

D'où une hypothèse testable et élégante :

> **Sous MPC, le filet stall/regression devient inutile.**

Rejoue la configuration divergente de ta Figure 6.7 (cube 0.5 kg, μ = 0.7, position A), filet désactivé. Si elle converge, tu as remplacé une heuristique par une propriété structurelle.

**La bonne façon de présenter tout ça dans ton mémoire :**

```
Ch. 4 réparé  →  relation force→twist disponible
                        │
        ┌───────────────┴───────────────┐
   QP ponctuel                      MPC horizon
   (contraintes                     (contraintes
    satisfaites                      anticipées)
    à chaque instant)                     │
        │                                 │
   livrable sûr                   extension, si le temps
```

Le QP n'est pas un lot de consolation : c'est le sous-cas à horizon 1 du MPC. Les présenter comme un continuum, avec le critère 3 comme charnière, fait de ton choix de méthode un **raisonnement** au lieu d'une préférence. C'est exactement ce que ton Chapitre 5 n'a pas aujourd'hui.

---

## 5. Ce que tu dois demander à ton superviseur

Ma septième question, à ajouter à la liste précédente, et probablement la plus utile de toutes :

> « J'ai appliqué six critères de sélection (nature des contraintes, force de contact comme variable explicite, disponibilité d'un modèle de prédiction, budget de calcul, redondance, géométrie du cône) et ils convergent vers un QP articulaire à horizon 1, parce que le modèle de prédiction du mouvement de l'objet me manque tant que le frottement de support n'est pas dans le Chapitre 4. **Est-ce que vous validez ce raisonnement, ou attendiez-vous d'emblée un MPC — auquel cas la réparation du Chapitre 4 devient la priorité absolue ?** »

Cette formulation te met en position d'ingénieur qui a raisonné, pas d'étudiant qui demande quoi faire. Et elle rend visible la dépendance réelle entre les deux chantiers, que ton superviseur n'a peut-être pas explicitée pour lui-même : **il te demande de réparer le Ch. 4 et il te demande du contrôle sous contraintes, et ces deux demandes sont liées.**
