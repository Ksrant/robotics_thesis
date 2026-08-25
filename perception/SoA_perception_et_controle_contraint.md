# État de l'art — perception & contrôle sous contraintes

> **Format imposé par la consigne.** Ton superviseur ne veut pas une revue de littérature. Il veut : *« regarder ce qu'ils ont fait, les limites/assumptions que je pourrais modifier/corriger simplement pour avoir ma plus-value. »*
>
> Chaque entrée ci-dessous suit donc la même structure : **ce qu'ils font → l'hypothèse qu'ils posent → ce que cette hypothèse coûte → ce que toi tu peux changer.** C'est aussi le remède à ce qu'il te reprochait au Ch. 3 (p15, p17) : un état de l'art qui ne débouche pas sur la méthode.
>
> **Avertissement.** J'ai identifié ces références par recherche, je ne les ai pas lues. Vérifie chaque affirmation avant de la citer — surtout les très récentes (les identifiants arXiv en `26xx` datent de 2026).

---

# Partie 1 — Perception

## 1.1 Pourquoi ton superviseur dit « check perception grasping »

Parce que la perception pour la **préhension** est mûre, et celle pour la **poussée** ne l'est pas. Il t'envoie chercher les outils là où ils existent. Mais il y a une différence structurelle à exploiter, et c'est là qu'est ton créneau :

| | Préhension | Poussée |
|---|---|---|
| Quand faut-il voir ? | **Avant** le contact, pour planifier la prise | **Pendant** tout le contact, sur plusieurs secondes |
| L'occlusion pendant le contact | Acceptée — la tâche est finie une fois saisi | **C'est le problème central** |
| Durée d'occlusion typique | < 1 s | 10–20 s dans ton benchmark |

Toute la perception pour la préhension pose implicitement que l'occlusion est brève et terminale. La poussée viole cette hypothèse en continu. **C'est l'hypothèse la plus facile à attaquer de tout ton état de l'art**, et elle légitime directement ton travail.

## 1.2 Les hypothèses à attaquer

### Hypothèse A — « la caméra est externe et voit toujours l'objet »

C'est l'hypothèse de la quasi-totalité des travaux de poussée, **y compris le tien** (ton §5.3 échantillonne une position vraie).

- **Contre-exemple utile :** *Learning Visuotactile Estimation and Control for Non-prehensile Manipulation under Occlusions* (arXiv 2412.13157) pose exactement ce diagnostic : les méthodes antérieures « avoid visual occlusions by relying on complex perception schemes external to the robot that guarantee continuous tracking ». Leur réponse : une caméra embarquée simple + du tactile pour traverser les occlusions prolongées.
- **Ce que ça coûte :** une caméra externe suppose une infrastructure calibrée dans la scène. Irréaliste en logistique ou en assistance domestique — précisément les applications que ton introduction invoque.
- **Ce que tu changes :** caméra **eye-in-hand**, donc pas d'infrastructure, pose connue par cinématique directe. Et surtout : l'occlusion cesse d'être un modèle inventé (ton tirage de Bernoulli à p = 0.9) pour devenir une **conséquence géométrique mesurable**.

### Hypothèse B — « le centroïde de ce qu'on voit est le centre de l'objet »

Très répandue, y compris dans les pipelines RGB-D basiques et dans ton propre script YOLO.

- **Ce que ça coûte :** une caméra unique ne voit qu'une surface. Pour ton cylindre R = 100 mm, le centroïde de l'arc visible est à 2R/π ≈ **63.7 mm** du vrai centre ; pour ton cube, à 50 mm. Ton seuil de succès vaut 25 mm. Biais systématique de 2 à 2.5 fois le seuil, qu'aucun filtrage ne réduit.
- **Ce que tu changes :** correction par ajustement de primitive connue (cercle de rayon R imposé pour le cylindre ; plan dominant pour le cube, qui rend en prime le **yaw observable**). En eye-in-hand, ce biais **tourne avec la caméra** — il devient une erreur variable dans le temps qui perturbe directement la direction de poussée, ce qu'une caméra fixe masque partiellement. Ton montage rend le problème plus visible, donc la correction plus utile.

### Hypothèse C — « perte de trames indépendante et identiquement distribuée »

C'est ton modèle actuel, et ton superviseur l'a démonté (p52) : `(1/30)/(1−p)` est un temps d'attente *moyen*, pas une borne pire cas — sous tirage de Bernoulli, il n'existe aucun maximum fini de trames consécutives perdues.

- **Ce que tu changes :** l'occlusion réelle par le bras est **corrélée dans le temps** (longues rafales pendant le contact) et **bornée par la géométrie**. Elle est donc à la fois plus sévère en durée typique et mieux bornée qu'un tirage i.i.d. Mesurer cette structure et la comparer au modèle i.i.d. corrige l'erreur de p52 **et** produit un résultat original.

### Hypothèse D — « on a besoin de voir en continu »

Celle-ci, tu l'as déjà attaquée sans le savoir. Ton §6.6.2 sépare « pousser précisément » de « s'arrêter précisément ». C'est ton meilleur résultat et il n'a reçu aucune critique.

- **Ce que l'eye-in-hand y apporte :** la sonde caméra (`camera_probe.py`) va très probablement montrer que la sphère pousseuse masque l'objet pendant le contact. Ton résultat cesse alors d'être une robustesse chanceuse pour devenir un **principe de conception validé par la géométrie du capteur** : la vision n'est pas disponible pendant le contact, le canal de force la remplace, et seul le critère d'arrêt doit être repensé. C'est exactement la structure de 2412.13157 (caméra embarquée + tactile), sauf que ton canal complémentaire est la force et non le tactile.

### Hypothèse E — « détecteur généraliste appris »

LINEMOD, PoseCNN, DenseFusion — que tu cites déjà — et les détecteurs type YOLO.

- **Ce que ça coûte :** ces méthodes supposent des objets texturés ou appartenant à des classes connues. Un cube rouge uni ou un cylindre lisse ne sont dans aucune classe COCO, et un détecteur pré-entraîné y échoue ou hallucine. Les fine-tuner exige un jeu de données annoté.
- **Ce que tu changes :** assume la simplicité. Objets géométriques connus, couleur contrôlée → segmentation HSV + ajustement de primitive fait mieux, en dix lignes, sans dataset. **Justifie ce choix explicitement** plutôt que de le subir : c'est un argument de conception, pas un aveu de faiblesse.

## 1.3 « Check a work image processing »

Il te demande une référence qui fasse du **vrai traitement d'image**, pour que ton Ch. 3 ne se contente plus de citer des méthodes que tu n'implémentes pas (son reproche p19 : *« none of these methods is implemented in the thesis »*).

Deux entrées de survol pour cadrer, à compléter par une lecture ciblée :

- *Robot Manipulation Based on Embodied Visual Perception: A Survey* (CAAI Trans. Intelligence Technology, 2025)
- *A Survey of Robotic Monocular Pose Estimation* (Sensors, 2025)

Et pour la vision active — pertinent puisque ta caméra bouge avec le bras : *Reinforcement Learning of Active Vision for Manipulating Objects under Occlusions* (arXiv 1811.08067), qui montre que des politiques coordonnant main et œil battent une caméra statique.

---

# Partie 2 — Contrôle sous contraintes

## 2.1 Le fait structurant : deux littératures qui ne se parlent pas

C'est le résultat le plus utile de ma recherche, et il te donne ton créneau.

**Littérature A — locomotion et contrôle corps-complet.** Le QP avec cône de frottement, limites de couple et limites articulaires y est *le standard depuis dix ans*. Contrôle du iCub sur contacts non coplanaires, contrôleurs corps-complet priorisés avec contraintes de contact, quadrupèdes manipulateurs, commande optimale sous cône de frottement. La méthode y est complètement mûre, et le cône y est systématiquement linéarisé en `|f_x| ≤ μ f_z`, `|f_y| ≤ μ f_z`.

**Littérature B — poussée non préhensile.** Elle traite le frottement par le **mode de contact** (collant / glissant-gauche / glissant-droit) dans un cadre MPC hybride : Hogan & Rodriguez (ton [9], *Reactive Planar Manipulation with Convex Hybrid MPC*), la planification contact-implicite à contraintes déclenchées par état, et plus récemment *Push Anything* (MPC contact-implicite) et *Push, Press, Slide* (modèles réduits conscients du mode).

**Ces deux littératures ne se croisent quasiment pas.** Le QP corps-complet sous contraintes est appliqué aux jambes, pas aux bras qui poussent. Et le contrôle de poussée traite le frottement par des modes hybrides coûteux, pas par une contrainte QP simple.

**Ton créneau est exactement là** : *transporter le formalisme QP sous contraintes, mûr en locomotion, vers la poussée non préhensile avec un bras redondant.* C'est une phrase que tu peux écrire dans ton Ch. 3 et défendre. Et elle répond mot pour mot à p18 (*« what non-trivial manipulator-specific problem do you solve — redundancy, joint limits, torque constraints, singularities ? »*) : ces problèmes sont *ceux que la littérature A résout déjà*, et que la littérature B ignore parce qu'elle travaille sur des bases mobiles ou des pousseurs idéalisés.

## 2.2 L'hypothèse la plus importante à attaquer : le mode de contact

Toute la littérature B s'organise autour de trois modes — collant, glissant à gauche, glissant à droite. **Ton contrôleur n'a aucune notion de mode.** Il commande une direction et laisse Drake résoudre ce qui se passe au contact. C'est précisément ce que ton superviseur pointe (p29) : *« if the controller is allowed to command forces outside this cone and Drake simply resolves the resulting slip, then this is a system property, not a constraint handled by your controller. »*

Or — et c'est le pont entre ton QP et le courant principal du domaine — **imposer le cône de frottement, c'est exactement s'engager dans le mode collant.** C'est la condition de *stable pushing* de Lynch & Mason, que tu cites au §3.2 sans l'utiliser.

Ça te donne une formulation propre de ta contribution :

> *Là où la littérature traite les modes de contact par une optimisation hybride avec variables entières, on montre qu'imposer le mode collant comme contrainte d'inégalité linéaire dans un QP convexe suffit pour la poussée quasi-statique planaire, à coût de calcul très inférieur.*

C'est vrai, c'est modeste, c'est défendable, et ça se teste.

## 2.3 Les hypothèses attaquables, une par une

| # | Hypothèse dans la littérature | Où | Ce que ça coûte | Ce que tu changes |
|---|---|---|---|---|
| F | Le pousseur est une base mobile omnidirectionnelle ou un point idéal | [19] Force Push, et l'essentiel de la littérature B | Ni redondance, ni butées, ni limites de couple : le contrôleur commande une vitesse cartésienne libre | Bras 7 DoF : ces contraintes deviennent actives, et c'est **le** problème spécifique manipulateur que p18 te réclame |
| G | Le cône de frottement est traité par modes hybrides (entiers) | Hogan & Rodriguez, contact-implicite | Coût combinatoire, replanification lourde | En planaire, `\|f_t\| ≤ μ f_n` est **exactement** deux inégalités linéaires — QP convexe, ~150 µs |
| H | La redondance est résolue hors du problème de contrainte | Littérature B | 4 dimensions non spécifiées sur ton Panda ; c'est p32 | Tâche de posture dans l'espace nul, **à l'intérieur** du même QP |
| I | Les limites de couple se gèrent par saturation | ton code actuel | La saturation change la **direction** du vecteur de couple, pas seulement sa norme ; c'est p32 | Contrainte dure `\|τ\| ≤ τ_max` dans le QP |
| J | Un modèle de mouvement de l'objet est disponible (limit surface) | toute la littérature MPC | Sans lui, pas de prédiction | **Tu ne l'as pas** — c'est le trou du Ch. 4 (p22). Il faut le réparer, et c'est ce qui déterminera si tu peux aller jusqu'au MPC |
| K | Le contact est ponctuel et sans moment | ton Ch. 4, hypothèse 4 | Ton superviseur note (p27) que le centre de la sphère n'est pas le point de contact | Petite correction, gratuite, qui montre de la rigueur |

**Lis la ligne J deux fois.** C'est la seule hypothèse de la liste que tu ne peux pas attaquer aujourd'hui — c'est toi qui es en défaut, pas la littérature. Elle relie tes deux chantiers : réparer le Ch. 4 conditionne jusqu'où ton contrôleur peut aller (QP à horizon 1 sans elle, MPC avec).

## 2.4 Références pour ancrer la section

**Côté QP / corps-complet** (à citer pour établir la maturité du formalisme) : contrôle corps-complet du iCub par régulation de force sur contacts rigides non coplanaires ; contrôleur corps-complet priorisé avec contraintes de contact ; contrôle corps-complet de manipulateurs quadrupèdes par QP ; commande optimale de robots à pattes sous cône de frottement ; contrôle de force corps-complet multi-contact pour robots pilotés en position.

**Côté poussée** (à citer pour situer le mode de contact et la limit surface) : Hogan & Rodriguez, MPC hybride convexe ; planification et contrôle contact-implicite à contraintes déclenchées par état ; *Push Anything* ; *Push, Press, Slide* ; surfaces limites asymétriques duales (Autonomous Robots, 2024) ; *Pushing Revisited* pour la platitude différentielle sous contact collant.

---

## Le tableau à mettre dans ton mémoire

Remplace ton Tableau 3.1 actuel — qui liste des approches sans conclure — par une version qui **débouche** sur ta méthode :

| Approche | Contraintes réellement imposées | Redondance | Coût | Ce que j'en retiens |
|---|---|---|---|---|
| Réactif force (Force Push [19]) | aucune | s.o. (base mobile) | négligeable | La loi de correction directionnelle |
| MPC hybride (Hogan & Rodriguez) | cône, via modes entiers | non traitée | élevé | La notion de mode de contact |
| QP corps-complet (locomotion) | cône + couples + butées | oui | faible | **Le formalisme** |
| **Ce travail** | **cône + couples + vitesses + butées** | **oui, tâche de posture** | **faible** | — |

Cette table seule répond à p15, p17 et p18.

---

## Ce que la sonde caméra va te dire (et pourquoi c'est la bonne première étape)

`camera_probe.py` monte la caméra sur `panda_hand`, compare trois positions de montage, et mesure à chaque instant combien de pixels de l'objet sont visibles. Regarde trois choses :

1. **La sphère pousseuse est-elle dans le champ ?** Elle fait 5 cm de rayon et elle est fixée à la même main que la caméra. Si elle occupe le centre de l'image, tout le reste en découle.
2. **L'objet reste-t-il visible pendant le contact ?** C'est la question qui décide de toute ta suite. Si la réponse est non — ce que je crois — alors la vision sert à l'approche et au repositionnement, la force sert au contact, et **seul le critère d'arrêt pose problème**. Ce qui est exactement ta conclusion du §6.6.2, mais expliquée au lieu d'observée.
3. **À quelle distance travaille-t-elle ?** Le bruit de profondeur d'une caméra stéréo croît en z². Travailler à 15 cm est très différent de travailler à 1 m — c'est un argument en faveur de l'eye-in-hand qu'il faut mesurer, pas supposer.

Le script imprime aussi l'axe de visée en coordonnées monde au premier tick, avec une alerte si la caméra regarde le plafond. **Vérifie ce vecteur avant d'interpréter la moindre image** : l'orientation de `panda_hand` dépend de ta configuration articulaire et je ne peux pas la deviner sans lancer ta simulation.

Un résultat négatif ici est un bon résultat. « La caméra eye-in-hand est aveugle pendant le contact, voici la mesure, voici ce que j'en déduis pour l'architecture » est une contribution ; « j'ai mis une caméra et ça marche » n'en est pas une.

---

**Sources** (identifiées par recherche, non lues — à vérifier avant citation) :

- [Learning Visuotactile Estimation and Control for Non-prehensile Manipulation under Occlusions](https://arxiv.org/pdf/2412.13157)
- [Uncertainty-Aware Non-Prehensile Manipulation with Mobile Manipulators under Object-Induced Occlusion](https://arxiv.org/pdf/2602.01731)
- [Reinforcement Learning of Active Vision for Manipulating Objects under Occlusions](https://arxiv.org/pdf/1811.08067)
- [Robot Manipulation Based on Embodied Visual Perception: A Survey](https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/cit2.70022)
- [A Survey of Robotic Monocular Pose Estimation](https://www.mdpi.com/1424-8220/25/5/1519)
- [Reactive Planar Manipulation with Convex Hybrid MPC](https://arxiv.org/pdf/1710.05724)
- [Contact-Implicit Planning and Control for Non-Prehensile Manipulation Using State-Triggered Constraints](https://arxiv.org/pdf/2210.09540)
- [Push Anything: Single- and Multi-Object Pushing From First Sight with Contact-Implicit MPC](https://arxiv.org/html/2510.19974)
- [Push, Press, Slide: Mode-Aware Planar Contact Manipulation via Reduced-Order Models](https://arxiv.org/html/2603.12399)
- [Dual asymmetric limit surfaces and their applications to planar manipulation](https://link.springer.com/article/10.1007/s10514-024-10173-5)
- [iCub Whole-Body Control through Force Regulation on Rigid Non-Coplanar Contacts](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2015.00006/full)
- [Computationally-Robust and Efficient Prioritized Whole-Body Controller with Contact Constraints](https://arxiv.org/pdf/1807.01222)
- [A Whole-Body Controller Based on a Simplified Template for Rendering Impedances in Quadruped Manipulators](https://arxiv.org/abs/2208.00810)
- [Optimal Control of Legged-Robots Subject to Friction Cone Constraints](https://arxiv.org/pdf/2208.02393)
- [Multi-Contact Whole-Body Force Control for Position-Controlled Robots](https://arxiv.org/html/2312.16465v4)
