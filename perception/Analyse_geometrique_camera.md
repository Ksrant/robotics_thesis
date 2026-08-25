# Analyse géométrique du montage caméra — avant de lancer quoi que ce soit

Trois choses ressortent de la lecture du URDF, dont une qui dépasse largement la question de la caméra.

---

## 1. Ce que le URDF résout

**`panda_hand` n'est pas une main : c'est la sphère pousseuse.**

```xml
<link name="panda_hand">
  <visual><origin xyz="0 0 0"/><geometry><sphere radius="0.05"/></geometry></visual>
  <collision><origin xyz="0 0 0"/><geometry><sphere radius="0.05"/></geometry></collision>
</link>
```

Pas de doigts, pas de pince. La sphère de 5 cm est centrée **exactement sur l'origine du repère**. Ça explique ton `FINGER_TIP_OFFSET = [0,0,0]` et ça impose une contrainte dure : toute position de montage doit vérifier `|xyz| > 0.05`, sinon la caméra est physiquement *à l'intérieur* du pousseur.

**L'orientation du repère, calculée par cinématique directe** à ta configuration initiale `q7 = [0, -0.3, 0, -2.2, 0, 2.0, 0]` :

```
panda_hand est à [0.474, 0, 0.516] dans le repère base

  +x_hand  ->  monde [ 0.995,  0,  0.100]     avant, quasi horizontal
  +y_hand  ->  monde [ 0,     -1,  0    ]
  +z_hand  ->  monde [ 0.100,  0, -0.995]     VERS LE BAS
```

Donc « avancer » = `+x_hand`, « monter » = `−z_hand`. C'est ce que mon script précédent ne pouvait pas deviner ; c'est maintenant fixé, et les `rpy` du bloc URDF en découlent.

Note au passage : ton `panda_hand_joint` a `rpy="0 0 0"` là où le Franka standard a `rpy="0 0 -0.7854"`. Tu as supprimé la rotation de 45°. Sans conséquence, mais à savoir si tu compares un jour à du matériel réel.

---

## 2. Ce que la géométrie prédit déjà — avant même de rendre une image

Ces chiffres sont calculables analytiquement, et ils constituent déjà une partie du livrable « voir ce que la caméra renvoie ». Autant les avoir avant de regarder les images, pour savoir quoi y chercher.

### La sphère masque-t-elle l'objet ?

Avec le montage « over » (caméra à `[0.02, 0, -0.10]`, visée vers `[0.125, 0, 0]`) :

- La ligne de visée passe à **8.6 cm** de l'axe de la sphère, dont le rayon vaut 5 cm → **l'objet n'est pas masqué**.
- La sphère occupe malgré tout le bas du champ : vue à 0.102 m, son rayon angulaire est `arcsin(0.05/0.102) = 29.4°`, et son centre est à 57.7° de l'axe optique. Elle s'étend donc de **28.3° à 87°** hors axe.
- Avec un demi-champ vertical de 29° (D455), son bord supérieur à 28.3° affleure tout juste le bas de l'image. Elle apparaîtra comme un liseré en bordure basse.

**Bonne nouvelle** — contrairement à ce que je te disais au message précédent, un montage bien choisi *ne condamne pas* la vue de l'objet pendant le contact. Mon hypothèse « la sphère masque tout » était trop pessimiste. À confirmer par le rendu, mais la géométrie dit que ça passe.

### Le vrai problème : l'objet est trop près, et trop gros

C'est l'inverse du problème que j'anticipais.

| | distance caméra→objet | rayon angulaire | demi-champ vertical | tient dans l'image ? |
|---|---|---|---|---|
| Cylindre (R = 100 mm), montage « over » | 0.145 m | **43.6°** | 29° | **non, largement** |
| Cube (demi-côté 50 mm), montage « over » | 0.128 m | **33.5°** | 29° | **non** |
| Cylindre, montage « over_back » | 0.195 m | 30.9° | 29° | presque |

Ton cylindre fait 200 mm de diamètre et la caméra le regarde à 145 mm. Il **déborde du cadre** pendant tout le contact. Conséquences directes :

- Le masque de segmentation touchera les bords de l'image → le centroïde de surface sera tronqué, et donc **biaisé d'une deuxième manière**, en plus du biais de surface visible dont je t'ai parlé. Il faudra détecter et rejeter les masques qui touchent le bord.
- L'ajustement de cercle à rayon connu devient *plus* utile, pas moins : c'est justement la méthode qui reste valide sur un arc partiel, là où un centroïde ne l'est pas.
- Le montage « over_back » est probablement le bon choix. Lance la sonde avec les trois et compare.

---

## 3. Le point vraiment important, qui n'est pas une question de caméra

**Tu ne contrôles pas l'orientation du poignet.**

Dans `_update`, tu ne commandes que des forces de translation :

```python
_, Jv = self._get_jacobians()      # Jv = J_full[:3, :]  -> translation seulement
...
send(Jv.T @ F_trans)
```

`Jv` est 3×7, donc `Jv.T @ F_trans` engendre un couple dans un sous-espace de dimension 3. **Les 4 dimensions restantes reçoivent un couple commandé nul**, et comme la gravité est compensée (`tau = tau_xyz - g_arm`), le mouvement dans l'espace nul n'est ni piloté ni amorti : il dérive librement, poussé seulement par les termes de Coriolis et la réaction de contact.

Tant que l'effecteur est une sphère, ça ne se voit pas — une sphère est invariante par rotation, l'orientation du poignet est sans effet sur la poussée. **Dès que tu y fixes une caméra, ça devient visible et bloquant** : la caméra pointe dans une direction qui dérive au cours de la poussée, et les `rpy` que je viens de calculer ne sont valides qu'à la configuration initiale.

### Pourquoi c'est une bonne nouvelle

C'est exactement le commentaire p32 de ton superviseur :

> « The Panda has 7 DoF, yet only the translational task is controlled here. **What happens to the remaining redundancy/orientation/null-space dynamics?** This is one of the non-trivial aspects of applying the Cartesian command to a redundant arm and is not discussed. »

Et c'est le **pont entre tes deux chantiers**. Monter une caméra sur le poignet t'oblige à contrôler son orientation ; contrôler son orientation t'oblige à résoudre la redondance ; résoudre la redondance est précisément la contribution de contrôle sous contraintes qu'il attend.

Tu passes donc de :

> « J'ai 7 DoF et une tâche 3D, et je ne dis rien des 4 dimensions restantes »

à :

> « J'ai 7 DoF et **deux** tâches — pousser en translation, et maintenir l'objet dans le champ de la caméra — hiérarchisées dans un QP sous contraintes. »

C'est une justification autrement plus solide du contrôleur corps-complet que « il faut bien imposer les limites de couple ». La caméra transforme l'espace nul d'oubli en tâche.

### Concrètement, dans le QP

Le terme de posture que je t'avais donné devient une **tâche de visée**. Au lieu de :

```python
qdd_post = KP_POST * (Q_POSTURE - q) - KD_POST * qd     # posture arbitraire
```

tu régules l'orientation du poignet pour que l'axe optique reste pointé vers l'objet :

```python
# erreur d'orientation : angle entre l'axe optique et la direction caméra->objet
z_opt   = X_WCam.rotation().matrix()[:, 2]
d_obj   = (p_obj_est - X_WCam.translation());  d_obj /= np.linalg.norm(d_obj)
e_rot   = np.cross(z_opt, d_obj)                # axe et amplitude de la correction
omega_des = K_AIM * e_rot
# -> résidu à minimiser dans le coût du QP :  ‖J_w q̈ + J̇_w q̇ − a_rot_des‖²
```

avec `J_w = J_full[3:, :]`, la partie rotationnelle de la jacobienne que tu calcules déjà et que tu jettes actuellement.

Le poids de cette tâche la place sous la tâche de poussée : la caméra suit l'objet **avec ce qu'il reste** de liberté après la poussée, ce qui est exactement la sémantique voulue.

---

## 4. Ordre des opérations

```
1. Coller le bloc URDF                                        (5 min)
2. Lancer camera_probe.py sur les 3 montages                  (30 min)
   -> regarder les images AVANT toute conclusion
   -> vérifier l'axe de visée imprimé au premier tick
3. Mesurer la dérive d'orientation du poignet                 (1 h)
   -> logger la matrice de rotation de panda_hand au cours d'un push
   -> C'EST LA FIGURE QUI JUSTIFIE TOUT LE RESTE
4. Choisir le montage, puis passer à la perception
```

L'étape 3 est celle que je te recommande le plus fortement, et elle ne coûte presque rien : logge `X_WHand.rotation()` à chaque tick et trace l'angle entre l'axe optique et la direction vers l'objet au cours de la poussée. Si cet angle dérive de plusieurs dizaines de degrés — ce que je crois, puisque rien ne le contraint — tu as en une figure :

- la démonstration que la caméra eye-in-hand **exige** de contrôler la redondance,
- la réponse à p32,
- et la justification du QP corps-complet, qui cesse d'être une préférence méthodologique pour devenir une nécessité de conception.

C'est la figure la plus rentable que tu puisses produire cette semaine.
