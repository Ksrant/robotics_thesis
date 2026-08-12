# Préparation à la défense — ce qu'il faut savoir expliquer

---

## 0. Le mémoire en trois phrases

> Je pousse un objet posé sur une table vers une position cible, avec un bras Franka Panda muni d'un effecteur sphérique, en utilisant la **direction de la force de contact** comme seul retour sur l'interaction, sans modèle de la dynamique de l'objet.
>
> Le contrôleur est une machine à états qui sépare l'approche, la poussée et le repositionnement, et qui commande une **vitesse cartésienne** suivie par une loi d'impédance.
>
> Je montre que la performance dépend surtout de la **géométrie de l'objet** et de **l'obliquité initiale** de la poussée, et que le contrôleur tolère la perte de la plupart des observations visuelles sans dégrader la trajectoire.

Si on ne te laisse dire qu'une chose : **le cube et le cylindre échouent pour des raisons opposées**, et c'est ce que tout ton chapitre 6 démontre.

---

## 1. Le modèle (chapitre 4) — ce qu'il faut pouvoir dériver au tableau

### La chaîne logique

1. **Six hypothèses** : corps rigide, contact plan avec la table, **un seul point de contact**, forces linéaires dans le plan uniquement, régime **quasi-statique**, frottement de Coulomb aux deux interfaces.

2. La matrice de préhension générale $F_o = G F_c$ **dégénère** au cas $n_c = 1$, $d = 2$ :
$$G = \begin{pmatrix} 1 & 0 \\ 0 & 1 \\ -r_y & r_x \end{pmatrix} \quad\Rightarrow\quad \tau_o = r_x f_t - r_y f_n$$

3. Sous l'hypothèse quasi-statique, le terme inertiel $\dot V_o$ est négligeable et l'équation de Newton-Euler **s'effondre en une relation algébrique** : c'est elle que le contrôleur réactif utilise, jamais l'équation dynamique complète.

### Le résultat central du chapitre : les deux formes ont des mécanismes de couple **opposés**

| | Cube | Cylindre |
|---|---|---|
| Point de contact | $r_c = (a,\, p_y)$, $p_y$ **libre** dans la face | $r_c = (R,\, 0)$, **fixé par la géométrie** |
| Couple | $\tau_o = a f_t - p_y f_n \approx -p_y f_n$ | $\tau_o = R f_t$ **exact** |
| Source du couple parasite | force **normale** décentrée | force **tangentielle** (frottement) |
| Rayon de giration | $c^2 = \tfrac{2}{3}a^2$ | $c^2 = \tfrac{1}{2}R^2$ |

**La phrase à retenir :** pour le cylindre, $f_n$ passe *par construction* par le centre de masse, donc ne produit aucun couple ; seul $f_t$ en produit. Pour le cube, c'est exactement l'inverse : c'est $f_n$ appliquée hors du centre de la face qui domine.

**Si on te demande pourquoi $\tau_o \approx -p_y f_n$ et pas la forme complète :** parce qu'on suppose $f_t$ petit devant $f_n$ pour une poussée alignée sur la face. C'est une **simplification de modélisation**, pas une identité — et tu le dis explicitement dans le texte. À $\mu = 0{,}7$ elle tient moins bien, et le terme $a f_t$ négligé redevient une source de rotation non corrigée. **C'est ton explication de la dégradation avec le frottement.**

**Dimensions** : cube demi-côté 0,05 m, cylindre rayon 0,10 m, hauteur 0,10 m, frottement table 0,5.

---

## 2. Le contrôleur (chapitre 5) — les décisions et *pourquoi*

### L'architecture

**Une FSM à quatre états** : APPROACH → PUSH → (REPOSITION) → DONE, servie par **deux lois de commande différentes**.

- **APPROACH / REPOSITION** : PD cartésien qui suit trois waypoints (hauteur de sécurité → descente → point de contact). Ce contrôleur n'a *aucune* notion de l'objet, du contact, ni même du but. Volontairement simple : rien de la tâche de poussée n'est pertinent avant le contact.
- **PUSH** : commande une **vitesse** $v_{\text{cmd}}$, suivie par impédance $F_{\text{trans}} = K_v(v_{\text{cmd}} - \dot p_s)$.

### La question du Jacobien — sois précis, on te la posera

Le Jacobien est utilisé **deux fois, jamais inversé** :
- **Direct** $J_v(q)$ pour obtenir la vitesse cartésienne de la sphère, $\dot p_s = J_v \dot q$, qui alimente le retour de la boucle d'impédance ;
- **Transposé** $J_v^\top$ pour convertir la force virtuelle en couple articulaire, $\tau = J_v^\top F_{\text{trans}} - g(q)$.

**Pourquoi pas $J_v^{-1}$ ?** Parce que commander une vitesse articulaire imposerait une trajectoire rigide. Passer par $J_v^\top$ garde l'effecteur **compliant** face à un objet dont on ne connaît pas la position exacte en temps réel. C'est l'argument de fond de toute l'architecture.

### La loi de Heins et ta contribution propre

$$\theta_p = \theta_d + (K_F + 1)\,\delta_f + K_C\,\delta_c$$

$\delta_f$ = écart angulaire entre la force mesurée et la direction désirée ; $\delta_c$ = décalage latéral de la sphère.

**Ta modification :** Heins replie la correction latérale dans le terme angulaire. Toi, tu ajoutes un **terme de vitesse latérale indépendant**, $v_{\text{lat}} = K_{\text{lat}}\,\varepsilon_{\text{lat}}\,\hat n_d$, **jamais atténué près du but**. C'est ta principale divergence par rapport à la référence, et l'ablation prouve qu'elle est nécessaire pour le cube.

### Les six décisions de conception à savoir justifier

C'est **là** que se joue « il sait de quoi il parle ». Chacune a été testée puis abandonnée ou retenue pour une raison observée.

| Décision | Pourquoi |
|---|---|
| **Face du cube gelée** au moment de la planification | Recalculée à chaque tick depuis $\hat d$ : si l'objet dépasse le but, $\hat d$ s'inverse et le contrôleur bascule sur la face opposée **sans que l'objet ait tourné**. Sauts de plusieurs centimètres observés. |
| **$\theta_o$ filtré** ($\Theta_o = 0{,}05$) avant de faire tourner cette face | $c^2 = \tfrac{2}{3}a^2$ : un cube plus petit tourne plus vite à couple égal. Sans filtre, une rotation rapide fait osciller $\hat n_{\text{face}}$ à chaque tick et contamine le centrage latéral **et** l'estimation géométrique. |
| **Filtre de direction plus lent que le filtre de force** (0,08 contre 0,30) | Deux échelles de temps différentes : le « flick » se joue sur ~1 s, la force sur ~1 ms. Un filtre rapide ne l'amortirait pas. |
| **$K_{\text{lat}}$ jamais atténué** près du but | Une version antérieure le réduisait à l'approche finale ; ça dégradait la précision latérale **exactement au moment où elle devait être maximale**. |
| **Fallback géométrique conditionné** à un test de proximité indépendant | Sans ce garde-fou, il rapportait une force quasi maximale (14 N = $K_c \times 0{,}02$) pendant plusieurs secondes **après** que la sphère se soit séparée de l'objet. Confirmé visuellement dans Meshcat. |
| **Direction de retrait recalculée à chaque tick** en phase DONE | Une direction figée n'a aucune garantie de pointer encore à l'opposé de l'objet si la géométrie a bougé dans les derniers ticks : elle peut ramener l'effecteur dans l'objet. |

### Les mécanismes de sécurité

- **Perte de contact** > 5 s → repositionnement, sauf si l'objet est déjà à moins de $2\,d_{\text{done}}$.
- **Stagnation** : pas de progrès de 1 mm pendant 4 s → repositionnement. Nécessaire parce que le profil de freinage ne réagit qu'à $|d|$, pas au **signe** du progrès : si l'objet s'éloigne, le profil remonte vers $V_{\text{push}}$ comme si une nouvelle poussée commençait.
- **Confirmation de contact** = union de trois signaux (solveur, test géométrique, $|f| > F_{\min}$).

---

## 3. Les résultats (chapitre 6) — les chiffres et ce qu'ils veulent dire

### Validation physique (à mentionner en premier, ça inspire confiance)

$F_{\text{theory}} = \mu_c m g$ avec $\mu_c$ moyenne harmonique. À 2 kg et $\mu = 0{,}3$ : **7,36 N prédits, 6–9 N observés**. Ce contrôle a aussi révélé que le pas de temps par défaut **surestimait les forces d'un facteur 2 à 3**, d'où le pas de $10^{-4}$ s utilisé partout ensuite. C'est un bon exemple de vérification faite *avant* d'interpréter quoi que ce soit.

### Grille : 2 formes × 4 masses × 3 frottements × 2 positions = 48 runs

**28/48 au total, 14/24 pour chaque forme.** Mais les motifs d'échec sont opposés :

- **Cylindre** : réussit partout à 0,5 et 1,0 kg, échoue à partir de 1,5 kg sauf à $\mu = 0{,}3$. **Aucune dépendance à la position** (7/12 en A comme en B). Logique : la normale de contact est recalculée à chaque tick, la direction de poussée n'est jamais structurellement désalignée. Le facteur limitant est la demande en effort.
- **Cube** : réussit partout à $\mu = 0{,}3$ (8/8). Les échecs apparaissent à $\mu \geq 0{,}5$ et, aux masses faibles, **exclusivement en position A**.

### Le résultat le plus fort : l'obliquité, quantifiée

| | Cube A (∼27°) | Cube B (∼11°) | Cyl. A | Cyl. B |
|---|---|---|---|---|
| Succès | 5/12 | 9/12 | 7/12 | 7/12 |
| Déviation latérale moy. | **31,3 mm** | **9,6 mm** | 10,6 | 5,9 |

**Le cube dévie 3,3× plus à forte obliquité ; le cylindre est insensible.** C'est le mécanisme du « dog-leg » : au début du contact sur une face plane, la force mesurée est quasi normale à cette face, donc $\delta_f$ vaut à peu près l'obliquité, et le terme $(K_F+1)\delta_f$ tire la direction commandée **vers la normale de la face, donc loin du but**. Il faut attendre qu'une composante tangentielle s'établisse pour que la correction cesse de s'opposer au progrès.

### Le résultat non monotone (celui qui montre que tu as regardé tes données)

À $\mu = 0{,}5$ et position A, la déviation latérale maximale **décroît régulièrement avec la masse** : 78,6 → 75,4 → 69,9 → 65,4 mm de 0,5 à 2,0 kg. Un objet lourd résiste mieux au couple parasite puisque $I_{zz} = mc^2$ croît avec la masse alors que la force normale décentrée, non. Le cube **s'améliore** donc jusqu'à 1,5 kg avant que la demande en effort ne prenne le dessus à 2,0 kg. Deux effets antagonistes dont le croisement définit l'enveloppe de fonctionnement.

### Ablation (0,25 kg, $\mu = 0{,}3$, positions C ∼14° et D ∼37°)

| Condition | C | D |
|---|---|---|
| Contrôleur complet | 25 mm | 39 mm |
| Sans filtre de direction | 25 mm | 38 mm |
| **Sans centrage latéral** | 25 mm | **107 mm** |

À faible obliquité, **aucun des deux termes ne sert**. À forte obliquité, le centrage latéral fait un facteur 2,7, le filtre de direction rien.

**Sur le filtre de direction, la bonne réponse :** il agit sur la *vitesse de rotation* de $\hat d$. Ces configurations convergent sans dépasser le but, donc $\hat d$ ne tourne jamais assez vite pour que filtré et non filtré diffèrent. **L'ablation n'exerce pas le régime que le filtre adresse.** Absence d'effet détecté ≠ terme inutile.

### Filet anti-stagnation

Sans lui : **890 mm**. Avec lui : **43 mm**. Le run échoue toujours (c'est une configuration que le cube rate même en perception parfaite), mais **la divergence est supprimée**. Distingue bien les deux : empêcher la divergence ≠ faire réussir la tâche.

### Perception — le résultat le plus subtil

| | Vérité terrain | Occl. 50 % | Occl. 90 % | Gelée |
|---|---|---|---|---|
| Succès | 8/8 | 0/8 | 4/8 | 1/8 |
| **Déviation latérale** | **15,8 mm** | **15,8 mm** | 28,3 | 38,0 |
| Erreur de perception | 0 | 4,0 | 6,6 | 199,6 |

**À 50 % de perte de trames, la déviation latérale est identique à la vérité terrain.** La trajectoire n'est pas dégradée du tout. Pourtant 0/8 réussissent, les huit finissant entre 27 et 32 mm.

**Le mécanisme :** le test d'arrêt compare la distance **perçue** au seuil. Avec 4 mm d'erreur, elle peut lire 24 mm alors que la vraie est à 28. L'erreur terminale vaut donc **seuil + erreur de perception**, ce que les mesures confirment (25 + 4 ≈ 28,4).

**La formule à retenir :** *« sous perception dégradée, le contrôleur ne pousse pas moins bien, il s'arrête moins bien. »*

Et le raisonnement de fond : l'erreur entre deux observations vaut $v\,\Delta t$. À 60 mm/s, perdre 90 % des trames laisse $\Delta t \approx 0{,}33$ s, soit ~20 mm. Le cas gelé est le point singulier $p = 1$, pas le régime d'une caméra réelle occultée.

---

## 4. Les questions qui vont venir, et quoi répondre

**« D'où vient la force, exactement ? »**
Du solveur de contact du simulateur, disponible **97 %** du temps ; l'estimation géométrique par pénétration couvre le reste. Ce canal joue le rôle qu'un capteur F/T ou un doigt tactile jouerait sur le robot réel. La thèse se place donc dans la commande compliante à retour d'effort, pas dans l'estimation sans capteur. *(Ne prétends pas l'inverse : c'est le point que tu risques le plus de te faire opposer.)*

**« Pourquoi toutes tes réussites sont exactement à 25 mm ? »**
Parce que le critère d'arrêt et le critère de succès partagent le même seuil : la distance finale d'un run réussi est **censurée par construction**. Elle dit que le contrôleur s'est arrêté où on le lui a demandé, pas avec quelle précision il aurait pu placer l'objet. C'est pour ça que je discrimine sur le **temps** et la **déviation latérale**.

**« Pourquoi 25 mm ? »**
Parce que la tolérance de waypoint du contrôleur de position vaut déjà 20 mm : descendre en dessous demanderait à la tâche de poussée plus de précision que le contrôleur de position n'en fournit lui-même. Et ça reste petit devant les objets (25 % du rayon du cylindre).

**« Un seul run par cellule ? »**
La simulation est déterministe : pas de temps fixe, état initial fixe, aucun terme stochastique dans le contrôleur. Répéter reproduit exactement. **Sauf** pour les campagnes caméra, où le bruit est tiré aléatoirement — c'est la limite que j'assume.

**« Ton ablation part d'une baseline qui échoue déjà. »**
Oui, à 39 mm contre un seuil de 25. Les géométries d'ablation ont été choisies délibérément au bout dur de la plage d'obliquité, là où la contribution de chaque terme est visible. La comparaison est donc entre un quasi-échec et un échec franc, pas entre un succès et un échec.

**« Tu compares à quoi ? »**
À rien d'externe, et c'est une limite. L'ablation est interne. Une comparaison à un pousseur en ligne droite sans retour d'effort serait le baseline le moins cher, c'est dans les travaux futurs.

**« Ça marcherait sur un vrai robot ? »**
Validation entièrement en simulation. Ce qui n'est pas capturé : la dynamique des actionneurs, l'estimation de force depuis les vrais couples articulaires, le matériel caméra, la compliance non modélisée du bras. Et le quasi-statique suppose 60 mm/s.

**Si on creuse le code :** `_current_face_offset()` suppose la normale de face alignée sur les axes du monde ; dès que le cube a tourné, elle surestime la distance centre-face jusqu'à 41 % à 45°. Le fallback géométrique en hérite. Comme il ne sert que 3 % du temps, l'effet reste borné — mais mieux vaut le dire toi-même que te le faire trouver.

---

## 5. Les trois choses à ne pas oublier de dire

1. **Le cube et le cylindre échouent pour des raisons opposées**, et je le montre à la fois par le modèle (couple normal décentré contre couple tangentiel) et par les mesures (déviation latérale 3,3× à forte obliquité pour le cube, insensibilité du cylindre).

2. **Chaque terme correctif a été introduit en réponse à un comportement observé**, pas ajouté par principe — et l'ablation montre qu'aucun des deux ne sert à faible obliquité.

3. **La perception dégradée ne casse pas la commande, elle casse la décision d'arrêt** — et j'ai le chiffre qui le prouve : déviation latérale identique à la vérité terrain à 50 % de perte de trames.
