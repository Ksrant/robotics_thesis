# Plan d'exécution

**L'objectif final :** deux colonnes dans un tableau.

| | Erreur finale de poussée | % du temps où l'objet est dans le cadre |
|---|---|---|
| Sans tâche de visée | *X* mm | faible |
| Avec tâche de visée | *X* mm — identique | élevé |

Tout ce qui suit sert à pouvoir remplir ce tableau honnêtement.

---

## Phase 0 — Prouver que le problème existe *(½ jour)* ⚠ **PORTE**

Le URDF est déjà fait. Logge, à chaque tick d'un push :

```python
X_WC   = plant.CalcRelativeTransform(ctx, plant.world_frame(),
                                     plant.GetFrameByName("camera_body"))
z_opt  = X_WC.rotation().matrix()[:, 2]      # attention: axe optique = +x de camera_body
d_obj  = object_xy_true - X_WC.translation()[:2]
alpha  = np.degrees(np.arccos(np.dot(z_opt[:2]/np.linalg.norm(z_opt[:2]),
                                     d_obj/np.linalg.norm(d_obj))))
```

Trace `α(t)`.

**Porte GO/NO-GO.** Si `α` reste stable, l'orientation ne dérive pas et **il n'y a pas de problème à résoudre** — arrête-toi et reviens me voir. Si `α` dérive de plusieurs dizaines de degrés, tu as ta Figure 1 et toute la suite est justifiée.

→ *Produit dans le mémoire :* **Figure 1** — « l'orientation du poignet n'est pas contrôlée ». C'est aussi la réponse directe au commentaire p32.

---

## Phase 1 — Voir ce que la caméra renvoie *(1 jour)*

C'est la consigne explicite de ton superviseur. Lance `camera_probe.py` sur les trois montages.

Trois choses à regarder :

1. L'axe de visée imprimé au premier tick — **avant** d'interpréter la moindre image.
2. Les images elles-mêmes. La sphère est-elle où le calcul le prédit (liseré en bas) ?
3. La courbe de visibilité par montage → **choisis le montage**.

Ma prédiction : `over_back` gagne, parce que le cylindre déborde du cadre avec `over` (43.6° contre 29° de demi-champ).

→ *Produit :* **Figure 2** — visibilité vs temps, par montage. Justifie ton choix de conception au lieu de le subir.

---

## Phase 2 — Réparer le Chapitre 4 *(1–2 jours)*

Prérequis au QP : on ne peut pas imposer en contrainte un cône dont les repères et les signes sont faux.

- Écrire le bilan de forces de l'objet **avec le torseur de frottement de support** (p22), en précisant que le contrôleur ne l'utilise pas de façon prédictive.
- Rotation de repère manquante dans Eq. (4.4) (p23).
- Signes entre Eqs. (4.15) et (4.16) (p27).
- Phrase de convention sur la gravité (p31).

**Pas de limit surface.** Elle n'est nécessaire que pour prédire, donc seulement pour un MPC, qui est écarté.

---

## Phase 3 — Le QP sous contraintes, couche 1 *(4–5 jours)*

Le livrable garanti, et l'objectif convenu manquant. **Pas encore de visée.**

Variables `(q̈, τ, f, s)`, égalité de la dynamique, et en contraintes : cône de frottement, couples, vitesses et butées articulaires. Le `np.clip` de `send()` disparaît.

Deux pièges déjà identifiés : ne construis pas le `MathematicalProgram` à 1 kHz (décime à 200–250 Hz), et hiérarchise dur/souple pour la faisabilité.

→ *Produit :* **Tableau 1** — comparaison [19] nu / ton contrôleur actuel / QP. Avec le taux de temps hors cône de frottement et le nombre de saturations de couple, deux diagnostics que ton mémoire actuel ne peut pas produire.

---

## Phase 4 — La visée, sur position vraie *(2 jours)* ⚠ **PORTE**

**Le point de méthode le plus important du plan : teste le mécanisme de visée en visant la position VRAIE de l'objet, pas encore la position perçue.**

Si tu branches perception et visée en même temps et que ça échoue, tu ne sauras pas lequel des deux a cassé. En visant la vérité terrain, tu valides le mécanisme d'espace nul **isolément**.

- Ajoute `J_a = −S(ẑ)[ẑ]_× J_ω` au coût du QP (`J_ω` est le bloc que `_get_jacobians()` calcule déjà et jette).
- Calcule et trace `σ_min(J_a N)` le long de la trajectoire.

**Porte GO/NO-GO.** Si `σ_min` reste franchement au-dessus de zéro et que `α(t)` s'effondre, ton affirmation tient. Si `σ_min` s'écroule, l'espace nul est insuffisant — l'innovation devient alors *une caractérisation de quand ça ne marche pas*, ce qui reste publiable mais qu'il faut savoir tôt.

→ *Produit :* **Figure 3** (`α` avec/sans) et **Figure 4** (`σ_min`, la prédiction théorique confirmée par la mesure).

---

## Phase 5 — La perception réelle *(3 jours)*

Maintenant seulement, le pipeline remplace `CameraModel`.

`LeafSystem` séparé à 30 Hz (jamais de rendu à 1 kHz), segmentation HSV, déprojection, ajustement de cercle / de plan. Valide `e_raw` contre `e_fit`.

→ *Produit :* **Figure 5** — erreur avec et sans correction de biais. C'est la figure qui montre que l'approche naïve rate la tâche d'un facteur 2.5 à 3.

---

## Phase 6 — Boucler *(1 jour)*

La visée pointe désormais la position **estimée**. C'est la boucle perception → contrôle complète.

---

## Phase 7 — Évaluation *(2 jours + temps machine)*

- N ≥ 10 graines par configuration.
- TOST sur l'erreur finale (marge δ = 5 mm), test de différence sur la visibilité.
- Les deux ensemble, ni l'un ni l'autre seul.

→ *Produit :* **le tableau de la première page.**

---

## Récapitulatif

```
Phase 0  ½ j   dérive d'orientation        ⚠ PORTE
Phase 1  1 j   sonde caméra, choix montage
Phase 2  1-2 j réparation Ch. 4
Phase 3  4-5 j QP sous contraintes         ← livrable garanti
Phase 4  2 j   visée sur position vraie    ⚠ PORTE  ← l'innovation
Phase 5  3 j   perception réelle
Phase 6  1 j   bouclage
Phase 7  2 j   évaluation statistique
────────────────────────────────────────
        ~15-17 jours de travail + temps machine
```

**Les deux portes existent pour t'éviter de brûler trois semaines sur une idée qui ne tient pas.** Passe-les avant d'investir dans la suite.

**Si tu manques de temps :** les phases 0 à 3 suffisent à sauver le mémoire, puisqu'elles livrent l'objectif convenu manquant. Les phases 4 à 7 sont l'innovation. Ne sacrifie jamais 0–3 pour aller plus vite vers 4.

---

## À valider avec ton superviseur avant de commencer

Il t'a demandé la caméra puis la perception. Ce plan intercale le QP (phase 3) entre la sonde caméra et le pipeline de perception, pour deux raisons : le QP est l'objectif convenu manquant, donc prioritaire ; et la visée doit être testée sur position vraie avant d'y ajouter le bruit de perception.

**Demande-lui si cet ordre lui convient.** S'il préfère la perception complète d'abord, échange les phases 3 et 5 — le plan reste valide, seule la protection contre le manque de temps s'affaiblit.
