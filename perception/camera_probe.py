"""
camera_probe.py
═══════════════════════════════════════════════════════════════════════════════
ÉTAPE 1 — "Simuler la caméra dans Drake, la placer sur le robot, et voir ce
qu'elle renvoie."

Ce script ne fait PAS de perception. Il répond à une seule question :
qu'est-ce qu'une caméra montée sur le poignet du Panda voit réellement
pendant une poussée ?

Il compare plusieurs positions de montage et produit, pour chacune :
  probe_out/<mount>/rgb_XXX.png      images couleur à intervalles réguliers
  probe_out/<mount>/depth_XXX.png    profondeur colorisée
  probe_out/<mount>/label_XXX.png    segmentation vérité terrain
  probe_out/<mount>/visibility.csv   pixels objet / pixels sphère par frame
  probe_out/visibility_compare.pdf   LA figure : visibilité vs temps, par montage

USAGE
─────
  python camera_probe.py --shape cylinder
  python camera_probe.py --shape cube --mounts wrist_down,wrist_side

CE QU'IL FAUT REGARDER
──────────────────────
1. La sphère pousseuse (rayon 5 cm, fixée à panda_hand) est-elle dans le champ,
   et quelle fraction occupe-t-elle ?
2. L'objet reste-t-il visible PENDANT le contact, ou disparaît-il derrière
   la sphère au moment précis où on en aurait besoin ?
3. À quelle distance travaille la caméra ? (le bruit de profondeur d'une
   RealSense croît en z², donc travailler à 15 cm n'a pas le même sens
   qu'à 1 m)

C'est la réponse à (2) qui déterminera toute la suite de ta perception.
"""

import os
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.geometry import (
    MakeRenderEngineVtk, RenderEngineVtkParams, ClippingRange, DepthRange,
    RenderCameraCore, ColorRenderCamera, DepthRenderCamera,
)
from pydrake.systems.sensors import RgbdSensor, CameraInfo
from pydrake.systems.analysis import Simulator

# Réutilise TA scène existante — rien n'est dupliqué ici.
from push_camera import create_sim_scene, write_sdf_if_needed  # cf. note en bas

RENDERER = "vtk"
OUT_ROOT = "probe_out"

# ═══════════════════════════════════════════════════════════════════════════
#  POSITIONS DE MONTAGE CANDIDATES
# ═══════════════════════════════════════════════════════════════════════════
# Poses exprimées DANS LE REPÈRE panda_hand.
# Convention optique Drake pour le repère caméra : +z = axe de visée,
# +x = droite de l'image, +y = bas de l'image.
#
# ATTENTION : l'orientation de panda_hand dépend de ta configuration
# articulaire. Ne devine pas — le script imprime l'axe de visée exprimé
# en coordonnées MONDE au premier tick. Vérifie ce vecteur avant
# d'interpréter quoi que ce soit. S'il pointe vers le plafond, corrige
# la RollPitchYaw ci-dessous.
#
# rpy : orientation du repère optique par rapport à panda_hand
# xyz : position du centre optique dans panda_hand
MOUNTS = {
    # Poignet, regardant vers l'avant-bas, reculée pour dégager la sphère
    "wrist_down": RigidTransform(
        RollPitchYaw(np.deg2rad([-30.0, 0.0, 0.0])).ToRotationMatrix(),
        np.array([0.0, -0.08, -0.06])),

    # Poignet, décalée latéralement : regarde l'objet "à côté" de la sphère
    "wrist_side": RigidTransform(
        RollPitchYaw(np.deg2rad([-20.0, 0.0, 35.0])).ToRotationMatrix(),
        np.array([0.09, -0.05, -0.04])),

    # Très en retrait : champ large, sphère petite dans l'image
    "wrist_back": RigidTransform(
        RollPitchYaw(np.deg2rad([-45.0, 0.0, 0.0])).ToRotationMatrix(),
        np.array([0.0, -0.14, -0.12])),
}


def build_camera(builder, plant, sg, X_HandCam, width=640, height=480):
    """Ajoute un RgbdSensor SOLIDAIRE de panda_hand (eye-in-hand)."""
    if not sg.HasRenderer(RENDERER):
        sg.AddRenderer(RENDERER, MakeRenderEngineVtk(RenderEngineVtkParams()))

    intrinsics = CameraInfo(width=width, height=height, fov_y=np.pi / 3)
    core = RenderCameraCore(RENDERER, intrinsics,
                            ClippingRange(0.02, 3.0),
                            RigidTransform())          # X_BS = I  →  C ≡ B
    color_cam = ColorRenderCamera(core, show_window=False)
    depth_cam = DepthRenderCamera(core, DepthRange(0.02, 3.0))

    # ── LA différence avec une caméra fixe : parent_id = le poignet
    hand_body = plant.GetBodyByName("panda_hand")
    hand_fid = plant.GetBodyFrameIdOrThrow(hand_body.index())

    cam = builder.AddNamedSystem("rgbd_eih", RgbdSensor(
        parent_id=hand_fid, X_PB=X_HandCam,
        color_camera=color_cam, depth_camera=depth_cam))
    builder.Connect(sg.get_query_output_port(), cam.query_object_input_port())
    return cam, intrinsics


def object_and_hand_labels(plant, sg, object_model_name="object"):
    """RenderLabel de l'objet et des corps de la main, pour le diagnostic.

    L'image de labels sert UNIQUEMENT au diagnostic et à la validation —
    jamais à alimenter un contrôleur. C'est ce qui distingue une mesure
    d'un oracle, et il faudra l'écrire explicitement dans le mémoire.
    """
    insp = sg.model_inspector()
    obj_labels, hand_labels = set(), set()

    def labels_of(body):
        out = set()
        for gid in plant.GetVisualGeometriesForBody(body):
            props = insp.GetPerceptionProperties(gid)
            if props is not None and props.HasProperty("label", "id"):
                out.add(int(props.GetProperty("label", "id")))
        return out

    obj_model = plant.GetModelInstanceByName(object_model_name)
    for bi in plant.GetBodyIndices(obj_model):
        b = plant.get_body(bi)
        if b.name() != "world":
            obj_labels |= labels_of(b)

    for name in ("panda_hand", "panda_link8", "panda_link7"):
        try:
            hand_labels |= labels_of(plant.GetBodyByName(name))
        except RuntimeError:
            pass

    return obj_labels, hand_labels


def run_probe(mount_name, X_HandCam, shape, sdf_path, half_extent,
              obj_pos, target, sim_time=20.0, n_snapshots=12):
    """Lance une poussée avec la caméra montée, échantillonne et mesure."""
    from pydrake.systems.framework import DiagramBuilder

    out_dir = os.path.join(OUT_ROOT, mount_name)
    os.makedirs(out_dir, exist_ok=True)

    # ── On reconstruit la scène en injectant la caméra.
    #    (create_sim_scene doit accepter un hook builder ; voir la note
    #     "INTÉGRATION" en bas de ce fichier si ce n'est pas encore le cas)
    diagram, logger, q7, ctrl, cam, intrinsics, plant, sg = create_sim_scene(
        sdf_path=sdf_path, shape_type=shape, half_extent=half_extent,
        object_pos=obj_pos, object_target=target,
        camera_model=None, render=False,
        camera_hook=lambda b, p, s: build_camera(b, p, s, X_HandCam),
    )

    obj_labels, hand_labels = object_and_hand_labels(plant, sg)
    print(f"[{mount_name}] labels objet={sorted(obj_labels)} "
          f"main={sorted(hand_labels)}")

    sim = Simulator(diagram)
    sim.Initialize()
    root = sim.get_mutable_context()
    cam_ctx = cam.GetMyContextFromRoot(root)

    rows = []
    times = np.linspace(0.5, sim_time, n_snapshots)
    first = True

    for k, t in enumerate(times):
        sim.AdvanceTo(t)
        cam_ctx = cam.GetMyContextFromRoot(sim.get_mutable_context())

        rgba = cam.color_image_output_port().Eval(cam_ctx).data
        depth = cam.depth_image_32F_output_port().Eval(cam_ctx).data[:, :, 0]
        label = cam.label_image_output_port().Eval(cam_ctx).data[:, :, 0]
        X_WC = cam.body_pose_in_world_output_port().Eval(cam_ctx)

        if first:
            # ── LE contrôle à faire avant toute interprétation ──
            fwd = X_WC.rotation().matrix()[:, 2]   # +z optique, en monde
            print(f"[{mount_name}] X_WC.p = {np.round(X_WC.translation(), 3)}")
            print(f"[{mount_name}] axe de visée (monde) = {np.round(fwd, 3)}")
            if fwd[2] > 0.3:
                print(f"[{mount_name}] ⚠ la caméra regarde VERS LE HAUT — "
                      f"corrige la RollPitchYaw de ce montage.")
            first = False

        obj_px = int(np.isin(label, list(obj_labels)).sum())
        hand_px = int(np.isin(label, list(hand_labels)).sum())
        obj_mask = np.isin(label, list(obj_labels))
        valid = obj_mask & np.isfinite(depth) & (depth > 0)
        med_d = float(np.median(depth[valid])) if valid.any() else float("nan")

        rows.append({"t": t, "obj_px": obj_px, "hand_px": hand_px,
                     "total_px": label.size, "median_depth_m": med_d,
                     "state": ctrl._ctrl_state})

        plt.imsave(f"{out_dir}/rgb_{k:03d}.png", rgba[:, :, :3])
        dvis = np.where(np.isfinite(depth), depth, 0.0)
        plt.imsave(f"{out_dir}/depth_{k:03d}.png", dvis, cmap="viridis")
        plt.imsave(f"{out_dir}/label_{k:03d}.png", label, cmap="tab20")

        print(f"[{mount_name}] t={t:5.2f}s  state={ctrl._ctrl_state:11s} "
              f"objet={obj_px:6d}px ({100*obj_px/label.size:5.2f}%)  "
              f"main={hand_px:6d}px ({100*hand_px/label.size:5.2f}%)  "
              f"depth_med={med_d:.3f}m")

    import csv
    with open(f"{out_dir}/visibility.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shape", default="cylinder", choices=["cube", "cylinder"])
    ap.add_argument("--mounts", default=",".join(MOUNTS.keys()))
    ap.add_argument("--mass", type=float, default=1.0)
    ap.add_argument("--mu", type=float, default=0.3)
    args = ap.parse_args()

    os.makedirs(OUT_ROOT, exist_ok=True)

    # Position A de ton Tableau 6.1 : forte obliquité, le cas intéressant
    obj_pos, target = [0.35, 0.10], [0.55, 0.20]
    sdf_path, half_extent = write_sdf_if_needed(args.shape, args.mass,
                                                args.mu, obj_pos)

    all_rows = {}
    for name in args.mounts.split(","):
        name = name.strip()
        if name not in MOUNTS:
            print(f"montage inconnu : {name}")
            continue
        print(f"\n{'='*70}\n  MONTAGE : {name}\n{'='*70}")
        all_rows[name] = run_probe(name, MOUNTS[name], args.shape,
                                   sdf_path, half_extent, obj_pos, target)

    # ── LA figure : visibilité de l'objet au cours de la poussée
    fig, ax = plt.subplots(figsize=(9, 5))
    for name, rows in all_rows.items():
        t = [r["t"] for r in rows]
        pct = [100.0 * r["obj_px"] / r["total_px"] for r in rows]
        ax.plot(t, pct, marker="o", label=name)
    ax.set_xlabel("temps [s]")
    ax.set_ylabel("pixels objet visibles [% de l'image]")
    ax.set_title(f"Visibilité de l'objet — caméra eye-in-hand ({args.shape})")
    ax.axhline(0.0, color="k", lw=0.5)
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{OUT_ROOT}/visibility_compare.pdf")
    print(f"\n→ {OUT_ROOT}/visibility_compare.pdf")


if __name__ == "__main__":
    main()


# ═══════════════════════════════════════════════════════════════════════════
#  INTÉGRATION — deux petites modifications dans push_camera.py
# ═══════════════════════════════════════════════════════════════════════════
#
# 1) create_sim_scene() doit accepter un hook et rendre plus d'objets :
#
#      def create_sim_scene(..., camera_hook=None):
#          ...
#          cam, intrinsics = (None, None)
#          if camera_hook is not None:
#              cam, intrinsics = camera_hook(builder, plant, sg)
#          ...
#          return builder.Build(), logger, best_q7, ctrl, cam, intrinsics, plant, sg
#
#    (garde une signature de retour compatible pour tes appels existants,
#     par ex. via un flag return_extras=False par défaut)
#
# 2) write_sdf_if_needed() : ta fonction write_sdf() vit dans
#    benchmark_august.py. Soit tu l'importes de là, soit tu la déplaces
#    dans un module commun. Profites-en pour ajouter la couleur de l'objet,
#    dont la segmentation aura besoin ensuite :
#
#      <visual name="visual">
#        <geometry>...</geometry>
#        <material><diffuse>0.9 0.1 0.1 1.0</diffuse></material>
#      </visual>
