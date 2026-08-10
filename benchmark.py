"""
benchmark.py
═══════════════════════════════════════════════════════════════════════════════
Automatically runs N trials of push_controller.run_trial() across all
combinations of mass x friction x initial position x shape, and produces:

  results/benchmark_raw.csv                — one row per run, all metrics
  results/benchmark_summary.tex            — LaTeX table ready for the thesis
  results/benchmark_success.pdf            — success rate heatmap
  results/benchmark_final_distance.pdf     — final position error heatmap
  results/benchmark_time.pdf               — completion time boxplots
  results/benchmark_drake_contact_pct.pdf  — Drake contact availability
  results/benchmark_noise_robustness.pdf   — robustness curve (if multiple
                                              noise/camera levels are merged)

USAGE
─────
  python benchmark.py                      # full grid (~72 runs)
  python benchmark.py --quick              # reduced grid (~8 runs)
  python benchmark.py --shape cube         # single shape only
  python benchmark.py --out my_results     # custom output folder
  python benchmark.py --analyse CSV        # re-read an existing CSV, regenerate figures only

REQUIREMENTS
────────────
  push_controller.py must be in the same folder.
  SDFs are generated on the fly into models_generated/ (see write_sdf()).

PERCEPTION NOISE — TWO INDEPENDENT MECHANISMS
────────────────────────────────────────────────
  --noise SIGMA (meters)
      Simple mechanism: monkey-patches PushController._get_object_xy() to
      add Gaussian noise N(0, sigma^2) directly on the ground-truth xy read
      from Drake. Cheap, but corrupts the ground-truth channel itself —
      final_dist_mm and lateral_dev_*_mm are no longer measuring true task
      performance once this is active, since "truth" and "perception" are
      no longer separable at that point. Kept for backward compatibility
      and quick sanity sweeps only.

  --camera
      Realistic mechanism (see camera_model.py): a CameraModel instance is
      passed into run_trial(), and push_controller.py explicitly tracks
      BOTH a ground-truth position (used for force measurement, contact
      detection, and all final metrics) and a perceived position (what the
      controller actually acts on — limited frame rate, latency, Gaussian
      noise). This is the mechanism to use for any experiment where you
      want to measure true task performance under imperfect perception.
      Tune with --camera-hz, --camera-latency, --camera-noise,
      --camera-occlusion.

  --noise and --camera are mutually exclusive (enforced below) — combining
  them would corrupt the ground-truth channel that --camera relies on to
  report true performance.
"""

import os
import sys
import csv
import time
import argparse
import itertools
import importlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from datetime import datetime

# ─── import the main controller module ───────────────────────────────────────
PUSH_MODULE = "cube_push_perc"   # must be in the same folder as this benchmark.py
try:
    push_mod = importlib.import_module(PUSH_MODULE)
    run_trial = push_mod.run_trial
    PushController = push_mod.PushController
except ImportError as e:
    print(f"[benchmark] Could not import {PUSH_MODULE}: {e}")
    print(f"  Check that {PUSH_MODULE}.py is in the same folder.")
    sys.exit(1)
try:
    from cameramodel2 import CameraModel
except ImportError:
    CameraModel = None   # --camera will fail with a clear message if used without this file

# ─── SDF paths ─────────────────────────────────────────────────────────────
SDF_PATHS = {
    "cube":     "models_generated/envcube_benchmark.sdf",
    "cylinder": "models_generated/envcylinder_benchmark.sdf",
}

# SDF template — filled in at runtime with the requested mass and friction
SDF_TEMPLATE_CUBE = """\
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="default">
    <model name="table">
      <static>true</static>
      <pose>0.2 0.0 0.025 0 0 0</pose>
      <link name="table_link">
        <visual name="v"><geometry><box><size>2 2 0.05</size></box></geometry>
          <material><diffuse>0.55 0.45 0.35 1.0</diffuse></material></visual>
        <collision name="c"><geometry><box><size>2 2 0.05</size></box></geometry>
          <surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
    <model name="object">
      <pose>{px} {py} {pz} 0 0 0</pose>
      <link name="object_link">
        <inertial>
          <mass>{mass}</mass>
          <inertia>
            <ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{ixx}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <visual name="v"><geometry><box><size>{side} {side} {height}</size></box></geometry>
          <material><diffuse>0.2 0.75 0.35 1.0</diffuse></material></visual>
        <collision name="c"><geometry><box><size>{side} {side} {height}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>"""

SDF_TEMPLATE_CYLINDER = """\
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="default">
    <model name="table">
      <static>true</static>
      <pose>0.2 0.0 0.025 0 0 0</pose>
      <link name="table_link">
        <visual name="v"><geometry><box><size>2 2 0.05</size></box></geometry>
          <material><diffuse>0.55 0.45 0.35 1.0</diffuse></material></visual>
        <collision name="c"><geometry><box><size>2 2 0.05</size></box></geometry>
          <surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
    <model name="object">
      <pose>{px} {py} {pz} 0 0 0</pose>
      <link name="object_link">
        <inertial>
          <mass>{mass}</mass>
          <inertia>
            <ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{izz}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <visual name="v">
          <geometry><cylinder><radius>{r}</radius><length>{length}</length></cylinder></geometry>
          <material><diffuse>0.2 0.45 0.85 1.0</diffuse></material></visual>
        <collision name="c">
          <geometry><cylinder><radius>{r}</radius><length>{length}</length></cylinder></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>"""


# ═══════════════════════════════════════════════════════════════════════════════
#  PARAMETER GRIDS
# ═══════════════════════════════════════════════════════════════════════════════

# Half-extent per shape (cube: half side, cylinder: radius) — values
# empirically validated (reasonable pusher/object ratio, cube CoM aligned,
# cylinder stable). SPHERE_RADIUS=0.05 in push_controller.
HALF_EXTENT_BY_SHAPE = {"cube": 0.05, "cylinder": 0.10}
TABLE_TOP_Z = 0.05   # must match push_controller.TABLE_TOP_Z

# Tightened positions to stay within FRANKA_MAX_REACH_XY (0.75m from
# ROBOT_BASE_XY=[-0.10,0]) even in the worst case (object arrived at
# target, cylinder half-extent 0.10m). All keep push_dir oriented +x, the
# only direction validated for reliable approach/contact.
GRID_FULL = {
    "shapes":    ["cube", "cylinder"],
    "masses":    [0.5, 1.0, 0.25, 1.5],          # kg
    "frictions": [0.3, 0.5, 0.7],
    "positions": [                               # (x, y) object, fixed target
        {"name": "A", "obj": [0.35,  0.05], "target": [0.55,  0.20]},
        {"name": "B", "obj": [0.35, -0.05], "target": [0.55, -0.20]},
        {"name": "C", "obj": [0.30,  0.10], "target": [0.55,  0.05]},
    ],
}

GRID_NOISE = {
    "shapes":    ["cube", "cylinder"],
    "masses":    [0.5],    # nominal mass — not the light object used for the
                            # controller ablation, and not heavy enough to
                            # trip torque saturation (Sec 6.4.1), which would
                            # otherwise confound a sigma sweep unrelated to it
    "frictions": [0.5],    # nominal friction, same reasoning
    "positions": [
        {"name": "A", "obj": [0.35, 0.05], "target": [0.55, 0.20]},
    ],
}

GRID_QUICK = {
    "shapes":    ["cube", "cylinder"],
    "masses":    [0.5, 2.0],
    "frictions": [0.3, 0.7],
    "positions": [
        {"name": "A", "obj": [0.35, 0.1], "target": [0.55, 0.2]},
    ],
}

SIM_TIME = 40.0   # s per run

GRID_ABLATION = {
    "shapes":    ["cube", "cylinder"],
    "masses":    [0.25],                         # lightest object: where the
                                                   # flick / drift failure modes
                                                   # were reported (Sec 6.4.2)
    "frictions": [0.3],                           # low friction: keeps torque
                                                   # saturation (Sec 6.4.1) out
                                                   # of the picture so the
                                                   # ablation isolates the
                                                   # near-goal instability
    "positions": [
        {"name": "A", "obj": [0.35,  0.05], "target": [0.55,  0.0]},  # oblique (~30 deg)
        {"name": "C", "obj": [0.35,  0.05], "target": [0.55,  0.2]},  # near face-aligned (~11 deg)
    ],
}


# ═══════════════════════════════════════════════════════════════════════════════
#  ON-THE-FLY SDF GENERATION
# ═══════════════════════════════════════════════════════════════════════════════

def _inertia_cube(mass, a):
    """I = 2/3 * m * a^2 for a solid cube of half-side a."""
    return 2.0 / 3.0 * mass * a**2

def _inertia_cylinder(mass, r, h):
    """Ix = Iy = m(3r^2+h^2)/12,  Iz = m*r^2/2."""
    ixx = mass * (3*r**2 + h**2) / 12.0
    izz = mass * r**2 / 2.0
    return ixx, izz

def write_sdf(shape, mass, mu, obj_pos, out_dir="models_generated"):
    os.makedirs(out_dir, exist_ok=True)
    fname = os.path.join(out_dir, f"bench_{shape}_m{mass}_mu{mu}.sdf")
    px, py_val = obj_pos[0], obj_pos[1]


    if shape == "cube":
        half_extent=0.05
        side = 2 * half_extent
        height = 0.10                 # cohérent avec le "0.1" du template
        pz = TABLE_TOP_Z + height / 2.0
        ixx = _inertia_cube(mass, half_extent)
        content = SDF_TEMPLATE_CUBE.format(
            px=px, py=py_val, pz=pz, mass=mass, ixx=ixx, side=side, height=height, mu=mu)
    else:
        half_extent = HALF_EXTENT_BY_SHAPE[shape]
        length = 0.10   # half-length 0.05 — cf. "cylinder standing up" decision
        pz = TABLE_TOP_Z + length / 2.0
        ixx, izz = _inertia_cylinder(mass, half_extent, length)
        content = SDF_TEMPLATE_CYLINDER.format(
            px=px, py=py_val, pz=pz, mass=mass, ixx=ixx, izz=izz,
            r=half_extent, length=length, mu=mu)

    with open(fname, "w") as f:
        f.write(content)
    return fname, half_extent


# ═══════════════════════════════════════════════════════════════════════════════
#  SIMPLE PERCEPTION NOISE PATCH (--noise)
# ═══════════════════════════════════════════════════════════════════════════════

_noise_std = 0.0   # set by --noise

_original_get_object_xy = None

def _install_noise_patch(noise_std: float):
    """
    Monkey-patches PushController._get_object_xy() to inject Gaussian
    noise N(0, noise_std^2) on the object's XY position. Simulates a
    coarse, imprecise perception WITHOUT the controller's ground-truth
    channel being preserved (see module docstring — use --camera instead
    if you need true-performance metrics under noise).
    """
    global _original_get_object_xy, _noise_std
    _noise_std = noise_std

    if _original_get_object_xy is None:
        _original_get_object_xy = PushController._get_object_xy

    def _noisy_get_object_xy(self, q):
        xy = _original_get_object_xy(self, q)
        if _noise_std > 0:
            xy = xy + np.random.normal(0, _noise_std, 2)
        return xy

    PushController._get_object_xy = _noisy_get_object_xy
    print(f"[benchmark] Perception noise patch active: sigma = {noise_std*1000:.1f} mm")


def _remove_noise_patch():
    global _original_get_object_xy
    if _original_get_object_xy is not None:
        PushController._get_object_xy = _original_get_object_xy
        _original_get_object_xy = None


# ═══════════════════════════════════════════════════════════════════════════════
#  CONTROLLER-COMPONENT ABLATION (--no-lateral-centering / --no-direction-filter)
# ═══════════════════════════════════════════════════════════════════════════════
#
# Unlike --noise/--camera (which corrupt PERCEPTION), these two flags
# disable pieces of the CONTROLLER itself (thesis Sec 5.2.1 "Direction
# Filtering" and Sec 5.2.4 "Lateral Centering"). They are implemented as
# direct patches on the imported push_mod's module-level constants
# (K_LAT, D_HAT_BETA), which _update() reads as bare globals on every
# tick (not as captured defaults) — so reassigning them on the module
# object takes effect immediately, exactly like the noise patch above.
#
# Ablation runs should be done WITHOUT --camera/--noise (ground-truth
# Drake position): the point is to isolate the controller's own
# contribution, not to mix it with a perception confound.

_orig_k_lat = None
_orig_d_hat_beta = None

def _install_ablation_patch(no_lateral_centering: bool, no_direction_filter: bool):
    global _orig_k_lat, _orig_d_hat_beta
    if _orig_k_lat is None:
        _orig_k_lat = push_mod.K_LAT
    if _orig_d_hat_beta is None:
        _orig_d_hat_beta = push_mod.D_HAT_BETA

    push_mod.K_LAT = 0.0 if no_lateral_centering else _orig_k_lat
    # D_HAT_BETA=1.0 means d_hat_filt <- 1.0*d_hat_raw + 0.0*d_hat_filt, i.e.
    # the low-pass filter of Eq. (5.6)/thesis Sec 5.2.1 collapses to the raw,
    # unfiltered direction every tick — the exact "recomputed every tick"
    # behaviour the thesis says was tried and abandoned for the cube, and
    # that produces the cylinder's "flick" near the goal.
    push_mod.D_HAT_BETA = 1.0 if no_direction_filter else _orig_d_hat_beta

    if no_lateral_centering:
        print("[benchmark] Ablation: lateral centering DISABLED (K_LAT = 0)")
    if no_direction_filter:
        print("[benchmark] Ablation: direction low-pass filter DISABLED "
              "(D_HAT_BETA = 1.0, raw d_hat every tick)")


def _remove_ablation_patch():
    global _orig_k_lat, _orig_d_hat_beta
    if _orig_k_lat is not None:
        push_mod.K_LAT = _orig_k_lat
        _orig_k_lat = None
    if _orig_d_hat_beta is not None:
        push_mod.D_HAT_BETA = _orig_d_hat_beta
        _orig_d_hat_beta = None


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def run_benchmark(grid: dict, out_dir: str, noise_std: float = 0.0,
                   shape_filter: str = None, camera_kwargs: dict = None,
                   no_lateral_centering: bool = False,
                   no_direction_filter: bool = False) -> list:

    os.makedirs(out_dir, exist_ok=True)

    if noise_std > 0:
        _install_noise_patch(noise_std)

    perception_mode = "camera" if camera_kwargs else ("noise" if noise_std > 0 else "absolute")

    # Build the run list
    runs = []
    for shape in grid["shapes"]:
        if shape_filter and shape != shape_filter:
            continue
        for mass in grid["masses"]:
            for mu in grid["frictions"]:
                for pos in grid["positions"]:
                    runs.append({
                        "shape":    shape,
                        "mass":     mass,
                        "mu":       mu,
                        "pos_name": pos["name"],
                        "obj_pos":  pos["obj"],
                        "target":   pos["target"],
                    })

    n_total = len(runs)
    print(f"\n{'='*60}")
    print(f"  BENCHMARK — {n_total} runs  |  perception={perception_mode}"
          f"{f' ({noise_std*1000:.1f}mm)' if perception_mode == 'noise' else ''}")
    print(f"  Output: {out_dir}")
    print(f"{'='*60}\n")

    results = []
    csv_path = os.path.join(out_dir, "benchmark_raw.csv")

    fieldnames = [
        "run_id", "shape", "mass_kg", "friction", "pos_name",
        "obj_x", "obj_y", "target_x", "target_y",
        "noise_mm",
        "perception_mode", "camera_hz", "camera_latency_ms", "camera_noise_mm",
        "no_lateral_centering", "no_direction_filter",
        "success", "completion_time_s", "final_dist_mm",
        "drake_contact_pct",
        "lateral_dev_mean_mm", "lateral_dev_max_mm",
        "perception_error_mean_mm", "perception_error_max_mm",
        "wall_time_s",
    ]
    with open(csv_path, "w", newline="") as f:
        csv.DictWriter(f, fieldnames=fieldnames).writeheader()

    for i, run in enumerate(runs):
        shape    = run["shape"]
        mass     = run["mass"]
        mu       = run["mu"]
        obj_pos  = run["obj_pos"]
        target   = run["target"]
        pos_name = run["pos_name"]

        print(f"[{i+1:02d}/{n_total}] {shape:8s}  m={mass}kg  mu={mu}  pos={pos_name}  "
              f"obj={obj_pos}  target={target}")

        sdf_path, half_extent = write_sdf(shape, mass, mu, obj_pos)
        pz = (TABLE_TOP_Z + half_extent if shape == "cube"
              else TABLE_TOP_Z + 0.05)   # cylinder: half-length 0.05

        run_tag = f"m{mass}_mu{mu}_{pos_name}"
        if noise_std > 0:
            run_tag += f"_noise{int(noise_std*1000)}mm"
        elif camera_kwargs:
            run_tag += f"_camhz{int(camera_kwargs['hz'])}_camnoise{int(camera_kwargs['noise_std_m']*1000)}mm"
        if no_lateral_centering:
            run_tag += "_noLat"
        if no_direction_filter:
            run_tag += "_noFilt"

        camera_model = CameraModel(**camera_kwargs) if camera_kwargs else None

        t_wall_start = time.time()
        try:
            metrics = run_trial(
                sdf_path          = sdf_path,
                shape_type        = shape,
                half_extent       = half_extent,
                object_model_name = "object",
                object_pos        = obj_pos + [pz],
                object_target     = target,
                sim_time          = SIM_TIME,
                render            = False,
                run_tag           = run_tag,
                camera_model      = camera_model,
            )
        except Exception as e:
            print(f"  X ERROR: {e}")
            metrics = {
                "shape": shape, "success": False,
                "final_dist_mm": float("nan"),
                "completion_time_s": None,
                "drake_contact_pct": 0.0,
                "lateral_dev_mean_mm": float("nan"), "lateral_dev_max_mm": float("nan"),
                "perception_error_mean_mm": float("nan"), "perception_error_max_mm": float("nan"),
            }
        wall_time = time.time() - t_wall_start

        ok   = metrics.get("success", False)
        dist = metrics.get("final_dist_mm", float("nan"))
        ct   = metrics.get("completion_time_s") or float("nan")
        pct  = metrics.get("drake_contact_pct", 0.0)
        status = "OK" if ok else "FAIL"
        print(f"  {status}  dist={dist:.1f}mm  t={ct:.1f}s  "
              f"Drake={pct:.1f}%  wall={wall_time:.1f}s")

        row = {
            "run_id":                    i + 1,
            "shape":                     shape,
            "mass_kg":                   mass,
            "friction":                  mu,
            "pos_name":                  pos_name,
            "obj_x":                     obj_pos[0],
            "obj_y":                     obj_pos[1],
            "target_x":                  target[0],
            "target_y":                  target[1],
            "noise_mm":                  noise_std * 1000,
            "perception_mode":           perception_mode,
            "camera_hz":                 camera_kwargs["hz"] if camera_kwargs else float("nan"),
            "camera_latency_ms":         camera_kwargs["latency_s"] * 1000 if camera_kwargs else float("nan"),
            "camera_noise_mm":           camera_kwargs["noise_std_m"] * 1000 if camera_kwargs else float("nan"),
            "no_lateral_centering":      int(no_lateral_centering),
            "no_direction_filter":       int(no_direction_filter),
            "success":                   int(ok),
            "completion_time_s":         ct,
            "final_dist_mm":             dist,
            "drake_contact_pct":         pct,
            "lateral_dev_mean_mm":       metrics.get("lateral_dev_mean_mm", float("nan")),
            "lateral_dev_max_mm":        metrics.get("lateral_dev_max_mm", float("nan")),
            "perception_error_mean_mm":  metrics.get("perception_error_mean_mm", float("nan")),
            "perception_error_max_mm":   metrics.get("perception_error_max_mm", float("nan")),
            "wall_time_s":               round(wall_time, 1),
        }
        results.append(row)

        # Incremental write (survives a Ctrl-C)
        with open(csv_path, "a", newline="") as f:
            csv.DictWriter(f, fieldnames=fieldnames).writerow(row)

    if noise_std > 0:
        _remove_noise_patch()

    print(f"\n[benchmark] CSV -> {csv_path}")
    return results


# ═══════════════════════════════════════════════════════════════════════════════
#  ANALYSIS AND FIGURES
# ═══════════════════════════════════════════════════════════════════════════════

plt.rcParams.update({
    "font.family": "serif", "font.size": 11,
    "axes.titlesize": 12, "axes.labelsize": 11,
    "legend.fontsize": 9, "figure.dpi": 150,
    "savefig.dpi": 300, "savefig.bbox": "tight",
    "axes.grid": True, "grid.alpha": 0.3,
    "axes.spines.top": False, "axes.spines.right": False,
})

SHAPE_COLORS = {"cube": "#2c7bb6", "cylinder": "#d7191c"}


def load_csv(csv_path: str) -> list:
    rows = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            for k in ["mass_kg", "friction", "noise_mm", "camera_hz",
                      "camera_latency_ms", "camera_noise_mm",
                      "completion_time_s", "final_dist_mm", "drake_contact_pct",
                      "lateral_dev_mean_mm", "lateral_dev_max_mm",
                      "perception_error_mean_mm", "perception_error_max_mm",
                      "wall_time_s"]:
                try:
                    row[k] = float(row[k]) if row[k] not in ("", "nan") else float("nan")
                except Exception:
                    row[k] = float("nan")
            row["success"] = int(row["success"])
            rows.append(row)
    return rows


def _get(rows, **filters):
    """Filter rows by the given kwargs criteria."""
    out = rows
    for k, v in filters.items():
        out = [r for r in out if str(r[k]) == str(v)]
    return out


# ── Figure 1: success rate heatmap ────────────────────────────────────────────

def plot_success_heatmap(rows, out_dir):
    shapes  = sorted(set(r["shape"]    for r in rows))
    masses  = sorted(set(r["mass_kg"]  for r in rows))
    fricts  = sorted(set(r["friction"] for r in rows))

    fig, axes = plt.subplots(1, len(shapes), figsize=(5 * len(shapes), 4),
                              squeeze=False)

    for col, shape in enumerate(shapes):
        ax   = axes[0][col]
        data = np.zeros((len(fricts), len(masses)))
        for ri, mu in enumerate(fricts):
            for ci, m in enumerate(masses):
                sub = _get(rows, shape=shape, mass_kg=m, friction=mu)
                if sub:
                    data[ri, ci] = 100.0 * np.mean([r["success"] for r in sub])
                else:
                    data[ri, ci] = float("nan")

        im = ax.imshow(data, vmin=0, vmax=100, cmap="RdYlGn", aspect="auto")
        ax.set_xticks(range(len(masses)));   ax.set_xticklabels([f"{m}" for m in masses])
        ax.set_yticks(range(len(fricts)));   ax.set_yticklabels([f"{mu}" for mu in fricts])
        ax.set_xlabel("Mass [kg]"); ax.set_ylabel("Friction mu")
        ax.set_title(f"{shape.capitalize()} — Success rate [%]")
        for ri in range(len(fricts)):
            for ci in range(len(masses)):
                v = data[ri, ci]
                if not np.isnan(v):
                    ax.text(ci, ri, f"{v:.0f}%", ha="center", va="center",
                            fontsize=10, color="black" if 20 < v < 80 else "white")
        plt.colorbar(im, ax=ax, label="%")

    plt.tight_layout()
    path = os.path.join(out_dir, "benchmark_success.pdf")
    plt.savefig(path); plt.close()
    print(f"  -> {path}")


def plot_final_distance(rows, out_dir):
    """
    Mean final position error [mm] per mass/friction — complements the
    binary success rate: a failure at 57mm and a failure at 140mm are both
    "0%" in benchmark_success.pdf, but very different in practice. This
    figure shows the continuous degradation.
    """
    shapes = sorted(set(r["shape"]    for r in rows))
    masses = sorted(set(r["mass_kg"]  for r in rows))
    fricts = sorted(set(r["friction"] for r in rows))

    fig, axes = plt.subplots(1, len(shapes), figsize=(5 * len(shapes), 4),
                              squeeze=False)

    for col, shape in enumerate(shapes):
        ax   = axes[0][col]
        data = np.full((len(fricts), len(masses)), np.nan)
        for ri, mu in enumerate(fricts):
            for ci, m in enumerate(masses):
                sub = _get(rows, shape=shape, mass_kg=m, friction=mu)
                dists = [r["final_dist_mm"] for r in sub if not np.isnan(r["final_dist_mm"])]
                if dists:
                    data[ri, ci] = np.mean(dists)

        vmax = np.nanmax(data) if np.any(~np.isnan(data)) else 100
        im = ax.imshow(data, vmin=0, vmax=vmax, cmap="RdYlGn_r", aspect="auto")
        ax.set_xticks(range(len(masses)));   ax.set_xticklabels([f"{m}" for m in masses])
        ax.set_yticks(range(len(fricts)));   ax.set_yticklabels([f"{mu}" for mu in fricts])
        ax.set_xlabel("Mass [kg]"); ax.set_ylabel("Friction mu")
        ax.set_title(f"{shape.capitalize()} — Final position error [mm]")
        for ri in range(len(fricts)):
            for ci in range(len(masses)):
                v = data[ri, ci]
                if not np.isnan(v):
                    ax.text(ci, ri, f"{v:.0f}", ha="center", va="center",
                            fontsize=10, color="black" if v < 0.5 * vmax else "white")
        plt.colorbar(im, ax=ax, label="mm")

    plt.tight_layout()
    path = os.path.join(out_dir, "benchmark_final_distance.pdf")
    plt.savefig(path); plt.close()
    print(f"  -> {path}")


# ── Figure 2: completion time boxplots ────────────────────────────────────────

def plot_completion_time(rows, out_dir):
    shapes = sorted(set(r["shape"] for r in rows))
    masses = sorted(set(r["mass_kg"] for r in rows))

    fig, axes = plt.subplots(1, len(shapes), figsize=(5 * len(shapes), 4),
                              squeeze=False, sharey=True)

    for col, shape in enumerate(shapes):
        ax = axes[0][col]
        data_by_mass = []
        labels       = []
        for m in masses:
            sub = _get(rows, shape=shape, mass_kg=m)
            times = [r["completion_time_s"] for r in sub
                     if r["success"] and not np.isnan(r["completion_time_s"])]
            data_by_mass.append(times if times else [float("nan")])
            labels.append(f"{m} kg")

        bp = ax.boxplot(data_by_mass, patch_artist=True,
                        medianprops=dict(color="black", lw=2))
        for patch in bp["boxes"]:
            patch.set_facecolor(SHAPE_COLORS[shape])
            patch.set_alpha(0.7)
        ax.set_xticklabels(labels)
        ax.set_xlabel("Mass [kg]"); ax.set_ylabel("Time [s]")
        ax.set_title(f"{shape.capitalize()} — Completion time (successful runs)")

    plt.tight_layout()
    path = os.path.join(out_dir, "benchmark_time.pdf")
    plt.savefig(path); plt.close()
    print(f"  -> {path}")


# ── Figure 3: Drake contact availability by shape/mass ────────────────────────

def plot_drake_contact_pct(rows, out_dir):
    """
    % of time the contact comes directly from Drake ContactResults (rather
    than the geometric fallback) — indicator of the quality of the contact
    actually established, by shape and mass.
    """
    shapes  = sorted(set(r["shape"]    for r in rows))
    masses  = sorted(set(r["mass_kg"]  for r in rows))

    fig, ax = plt.subplots(figsize=(7, 4))
    x       = np.arange(len(masses))
    width   = 0.35

    for si, shape in enumerate(shapes):
        means, stds = [], []
        for m in masses:
            sub = _get(rows, shape=shape, mass_kg=m)
            vals = [r["drake_contact_pct"] for r in sub
                    if not np.isnan(r["drake_contact_pct"])]
            means.append(np.nanmean(vals) if vals else float("nan"))
            stds.append(np.nanstd(vals)   if vals else 0.0)
        offset = (si - 0.5) * width
        ax.bar(x + offset, means, width, yerr=stds,
               label=shape.capitalize(), color=SHAPE_COLORS[shape],
               alpha=0.75, capsize=4)

    ax.set_xticks(x); ax.set_xticklabels([f"{m} kg" for m in masses])
    ax.set_ylabel("Drake contact availability [%]")
    ax.set_ylim(0, 105)
    ax.set_title("Drake contact availability — cube vs cylinder")
    ax.legend()
    plt.tight_layout()
    path = os.path.join(out_dir, "benchmark_drake_contact_pct.pdf")
    plt.savefig(path); plt.close()
    print(f"  -> {path}")


# ── Figure 4: robustness to noise/perception level (if several levels present) ─

def plot_noise_robustness(rows, out_dir):
    """
    Works for both perception mechanisms as long as several runs share the
    same out_dir with different noise_mm (--noise sweep) or camera_noise_mm
    (--camera sweep) values — pass a merged CSV via --analyse to compare.
    """
    level_col = "camera_noise_mm" if any(not np.isnan(r["camera_noise_mm"]) for r in rows) else "noise_mm"
    levels = sorted(set(r[level_col] for r in rows if not np.isnan(r[level_col])))
    if len(levels) < 2:
        return   # not enough levels to plot

    shapes = sorted(set(r["shape"] for r in rows))
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    for si, shape in enumerate(shapes):
        success_rates, dist_means = [], []
        for lvl in levels:
            sub = _get(rows, shape=shape, **{level_col: lvl})
            success_rates.append(100.0 * np.mean([r["success"] for r in sub]) if sub else float("nan"))
            dists = [r["final_dist_mm"] for r in sub if not np.isnan(r["final_dist_mm"])]
            dist_means.append(np.nanmean(dists) if dists else float("nan"))

        axes[0].plot(levels, success_rates, "o-",
                     color=SHAPE_COLORS[shape], lw=2, label=shape.capitalize())
        axes[1].plot(levels, dist_means, "s--",
                     color=SHAPE_COLORS[shape], lw=2, label=shape.capitalize())

    label = "Camera noise sigma [mm]" if level_col == "camera_noise_mm" else "Perception noise sigma [mm]"
    axes[0].set_xlabel(label)
    axes[0].set_ylabel("Success rate [%]")
    axes[0].set_title("Robustness to perception noise — success rate")
    axes[0].set_ylim(-5, 105); axes[0].legend()

    axes[1].set_xlabel(label)
    axes[1].set_ylabel("Final distance [mm]")
    axes[1].set_title("Robustness to perception noise — final accuracy")
    axes[1].legend()

    plt.tight_layout()
    path = os.path.join(out_dir, "benchmark_noise_robustness.pdf")
    plt.savefig(path); plt.close()
    print(f"  -> {path}")

def write_latex_table_condensed(rows, out_dir):
    """
    Version condensée pour le corps du mémoire — une seule ligne par
    (forme, mu), succès et distance combinés dans chaque cellule
    ("83% / 24mm"). Le détail complet (temps, Drake%) reste dans
    write_latex_table() pour l'annexe.
    """
    shapes = sorted(set(r["shape"] for r in rows))
    masses = sorted(set(r["mass_kg"] for r in rows))
    fricts = sorted(set(r["friction"] for r in rows))

    lines = [
        r"\begin{table}[ht]",
        r"  \centering",
        r"  \caption{Résultats condensés — taux de succès [\%] / erreur finale [mm]}",
        r"  \label{tab:benchmark_condensed}",
        r"  \begin{tabular}{ll" + "r" * len(masses) + "}",
        r"    \toprule",
        r"    Forme & $\mu$ & " + " & ".join(f"{m} kg" for m in masses) + r" \\",
        r"    \midrule",
    ]
    for shape in shapes:
        lines.append(f"    \\multirow{{{len(fricts)}}}{{*}}{{{shape.capitalize()}}}")
        for mu in fricts:
            vals = []
            for m in masses:
                sub = _get(rows, shape=shape, mass_kg=m, friction=mu)
                if sub:
                    sr = 100 * np.mean([r["success"] for r in sub])
                    dists = [r["final_dist_mm"] for r in sub if not np.isnan(r["final_dist_mm"])]
                    d = np.mean(dists) if dists else float("nan")
                    vals.append(f"{sr:.0f}\\% / {d:.0f}mm")
                else:
                    vals.append("-")
            lines.append("    & " + f"{mu}" + " & " + " & ".join(vals) + r" \\")
        lines.append(r"    \midrule")
    lines[-1] = r"    \bottomrule"
    lines += [r"  \end{tabular}", r"\end{table}"]
    path = os.path.join(out_dir, "benchmark_summary_condensed.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  -> {path}")

def write_position_table(rows, out_dir):
    """Vérifie la généralisation au-delà de la position canonique A —
    collapse sur masse/friction. Table d'annexe, 4 lignes pour 2 formes."""
    shapes = sorted(set(r["shape"] for r in rows))
    positions = sorted(set(r["pos_name"] for r in rows))
    lines = [
        r"\begin{table}[ht]", r"  \centering",
        r"  \caption{Généralisation par position initiale — moyenne sur masse/friction}",
        r"  \label{tab:benchmark_position}",
        r"  \begin{tabular}{ll" + "r" * len(positions) + "}",
        r"    \toprule",
        r"    Forme & Métrique & " + " & ".join(f"Pos. {p}" for p in positions) + r" \\",
        r"    \midrule",
    ]
    for shape in shapes:
        vals_s, vals_d = [], []
        for p in positions:
            sub = _get(rows, shape=shape, pos_name=p)
            vals_s.append(f"{100*np.mean([r['success'] for r in sub]):.0f}\\%" if sub else "-")
            dists = [r["final_dist_mm"] for r in sub if not np.isnan(r["final_dist_mm"])]
            vals_d.append(f"{np.mean(dists):.0f}mm" if dists else "-")
        lines.append(f"    \\multirow{{2}}{{*}}{{{shape.capitalize()}}} & Succès & " + " & ".join(vals_s) + r" \\")
        lines.append("    & Dist. finale & " + " & ".join(vals_d) + r" \\")
        lines.append(r"    \midrule")
    lines[-1] = r"    \bottomrule"
    lines += [r"  \end{tabular}", r"\end{table}"]
    path = os.path.join(out_dir, "benchmark_position_table.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  -> {path}")

# ── LaTeX table ─────────────────────────────────────────────────────────────
# NOTE: captions/labels below are in English per current request. This table
# is meant to be pasted directly into the (French) thesis — if you need
# French captions there, just translate the literal strings marked below;
# nothing else in the pipeline depends on their language.

def write_latex_table(rows, out_dir, noise_std=0.0):
    shapes  = sorted(set(r["shape"]    for r in rows))
    masses  = sorted(set(r["mass_kg"]  for r in rows))
    fricts  = sorted(set(r["friction"] for r in rows))

    lines = [
        r"\begin{table}[ht]",
        r"  \centering",
        r"  \caption{Benchmark results — success rate [\%], mean time [s], "
        r"final distance [mm], Drake contact availability [\%]}",
        r"  \label{tab:benchmark}",
        r"  \begin{tabular}{llr" + "r" * len(masses) + "}",
        r"    \toprule",
        r"    Shape & mu & Metric & " +
        " & ".join(f"{m} kg" for m in masses) + r" \\",
        r"    \midrule",
    ]

    for shape in shapes:
        lines.append(f"    \\multirow{{{4*len(fricts)}}}{{*}}"
                     f"{{{shape.capitalize()}}}")
        for mu in fricts:
            # success row
            vals_s = []
            for m in masses:
                sub = _get(rows, shape=shape, mass_kg=m, friction=mu)
                vals_s.append(f"{100*np.mean([r['success'] for r in sub]):.0f}"
                               if sub else "-")
            lines.append("    & " + f"{mu}" + " & Success [\\%] & " +
                          " & ".join(vals_s) + r" \\")

            # time row
            vals_t = []
            for m in masses:
                sub = _get(rows, shape=shape, mass_kg=m, friction=mu)
                times = [r["completion_time_s"] for r in sub
                         if r["success"] and not np.isnan(r["completion_time_s"])]
                vals_t.append(f"{np.mean(times):.1f}" if times else "-")
            lines.append("    & & Time [s] & " + " & ".join(vals_t) + r" \\")

            # final distance row
            vals_d = []
            for m in masses:
                sub = _get(rows, shape=shape, mass_kg=m, friction=mu)
                dists = [r["final_dist_mm"] for r in sub if not np.isnan(r["final_dist_mm"])]
                vals_d.append(f"{np.mean(dists):.1f}" if dists else "-")
            lines.append("    & & Final dist. [mm] & " + " & ".join(vals_d) + r" \\")

            # Drake availability row
            vals_f = []
            for m in masses:
                sub = _get(rows, shape=shape, mass_kg=m, friction=mu)
                pcts = [r["drake_contact_pct"] for r in sub
                        if not np.isnan(r["drake_contact_pct"])]
                vals_f.append(f"{np.mean(pcts):.1f}" if pcts else "-")
            lines.append("    & & Drake [\\%] & " + " & ".join(vals_f) + r" \\")

        lines.append(r"    \midrule")

    lines[-1] = r"    \bottomrule"
    lines += [r"  \end{tabular}", r"\end{table}"]

    path = os.path.join(out_dir, "benchmark_summary.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  -> {path}")


def write_latex_table_ablation(rows, out_dir):
    """
    Ablation table (new Results subsection): compares the full controller
    against variants with one controller component disabled, on
    GRID_ABLATION (light object, low friction, positions A vs C — see that
    grid's comment for why). Silently does nothing if the rows weren't
    produced with --no-lateral-centering / --no-direction-filter (e.g. an
    older CSV without these columns), or if only one condition is present.
    """
    if not rows or "no_lateral_centering" not in rows[0] or "no_direction_filter" not in rows[0]:
        return

    def condition_label(nlc, ndf):
        if nlc and ndf:
            return "No lateral centering + no direction filter"
        if nlc:
            return "No lateral centering (K\\_LAT=0)"
        if ndf:
            return "No direction filter (raw d\\_hat)"
        return "Full controller (baseline)"

    conditions = sorted(set((int(r["no_lateral_centering"]), int(r["no_direction_filter"]))
                             for r in rows))
    if len(conditions) < 2:
        return   # nothing to compare against

    shapes = sorted(set(r["shape"] for r in rows))
    lines = [
        r"\begin{table}[ht]",
        r"  \centering",
        r"  \caption{Ablation of two controller components --- success rate [\%] / "
        r"mean final distance [mm], light object (0.25\,kg), low friction ($\mu=0.3$), "
        r"positions A and C combined.}",
        r"  \label{tab:ablation}",
        r"  \begin{tabular}{l" + "r" * len(shapes) + "}",
        r"    \toprule",
        r"    Condition & " + " & ".join(sh.capitalize() for sh in shapes) + r" \\",
        r"    \midrule",
    ]
    for nlc, ndf in conditions:
        vals = []
        for shape in shapes:
            sub = [r for r in rows if r["shape"] == shape
                   and int(r["no_lateral_centering"]) == nlc
                   and int(r["no_direction_filter"]) == ndf]
            if sub:
                sr = 100 * np.mean([r["success"] for r in sub])
                dists = [r["final_dist_mm"] for r in sub if not np.isnan(r["final_dist_mm"])]
                d = np.mean(dists) if dists else float("nan")
                vals.append(f"{sr:.0f}\\% / {d:.0f}mm")
            else:
                vals.append("-")
        lines.append(f"    {condition_label(nlc, ndf)} & " + " & ".join(vals) + r" \\")
    lines.append(r"    \bottomrule")
    lines += [r"  \end{tabular}", r"\end{table}"]
    path = os.path.join(out_dir, "benchmark_ablation.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  -> {path}")


# ── Console summary ────────────────────────────────────────────────────────────

def print_summary(rows):
    shapes = sorted(set(r["shape"] for r in rows))
    print(f"\n{'='*60}")
    print("  BENCHMARK SUMMARY")
    print(f"{'='*60}")
    for shape in shapes:
        sub = [r for r in rows if r["shape"] == shape]
        sr  = 100.0 * np.mean([r["success"] for r in sub])
        times = [r["completion_time_s"] for r in sub
                 if r["success"] and not np.isnan(r["completion_time_s"])]
        pcts = [r["drake_contact_pct"] for r in sub if not np.isnan(r["drake_contact_pct"])]
        lat = [r["lateral_dev_mean_mm"] for r in sub if not np.isnan(r["lateral_dev_mean_mm"])]
        lat_str = f"  lateral_dev={np.mean(lat):.1f}mm" if lat else ""
        print(f"  {shape:10s}  success={sr:.0f}%  "
              f"mean_t={np.mean(times):.1f}s  "
              f"Drake={np.mean(pcts):.1f}%" + lat_str)
    print(f"{'='*60}\n")


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def analyse(csv_path: str, out_dir: str):
    print(f"\n[benchmark] Analysing {csv_path}")
    rows = load_csv(csv_path)
    print(f"  {len(rows)} runs loaded")
    print("[benchmark] Generating figures:")
    plot_success_heatmap(rows, out_dir)
    plot_final_distance(rows, out_dir)
    plot_completion_time(rows, out_dir)
    plot_drake_contact_pct(rows, out_dir)
    plot_noise_robustness(rows, out_dir)
    write_latex_table_condensed(rows, out_dir)
    write_position_table(rows, out_dir)
    write_latex_table(rows, out_dir)
    write_latex_table_ablation(rows, out_dir)
    print_summary(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Benchmark for the push controller")
    parser.add_argument("--quick",    action="store_true",
                        help="Reduced grid (~8 runs)")
    parser.add_argument("--noise",    type=float, default=0.0,
                        metavar="SIGMA",
                        help="Simple perception noise sigma in meters (e.g. 0.010 = 10mm). "
                             "Mutually exclusive with --camera.")
    parser.add_argument("--camera",   action="store_true",
                        help="Enable the realistic camera model (rate + latency + noise)")
    parser.add_argument("--camera-hz",        type=float, default=30.0,
                        help="Camera frame rate [Hz] (default 30)")
    parser.add_argument("--camera-latency",   type=float, default=0.08,
                        help="Camera processing latency [s] (default 0.08)")
    parser.add_argument("--camera-noise",     type=float, default=0.003,
                        help="Camera measurement noise std [m] (default 0.003)")
    parser.add_argument("--camera-occlusion", type=float, default=0.0,
                        help="Frame-drop probability while in contact (default 0.0)")
    parser.add_argument("--shape",    type=str, default=None,
                        choices=["cube", "cylinder"],
                        help="Run a single shape only")
    parser.add_argument("--out",      type=str, default="no_stall_results",
                        help="Output folder")
    parser.add_argument("--analyse",  type=str, default=None,
                        metavar="CSV",
                        help="Re-read an existing CSV and regenerate figures only")
    parser.add_argument("--no-lateral-centering", action="store_true",
                        help="Ablation: disable the lateral centering term "
                             "(K_LAT=0, thesis Sec 5.2.4). Use with "
                             "--ablation-grid to keep the run count small.")
    parser.add_argument("--no-direction-filter", action="store_true",
                        help="Ablation: disable the direction low-pass filter "
                             "(D_HAT_BETA=1.0 -> raw d_hat every tick, thesis "
                             "Sec 5.2.1). Use with --ablation-grid.")
    parser.add_argument("--ablation-grid", action="store_true",
                        help="Small, targeted grid for controller-component "
                             "ablations: 1 mass (0.25kg), 1 friction (0.3), "
                             "positions A & C, both shapes = 4 runs per "
                             "condition, instead of the full 36-per-shape grid.")
    parser.add_argument("--noise-grid", action="store_true",
                        help="Small, single-condition grid for the perception "
                             "(--camera) noise sweep: 1 mass (0.5kg, nominal), "
                             "1 friction (0.5, nominal), position A only = "
                             "2 runs/shape per noise level. Avoids mixing in "
                             "the torque-saturation regime of Sec 6.4.1.")
    parser.add_argument("--camera-noise-sweep", type=str, default=None,
                        metavar="S1,S2,...",
                        help="Comma-separated camera noise std values in "
                             "meters, e.g. '0,0.003,0.008,0.015,0.025'. Runs "
                             "--camera once per level, merges every level "
                             "into ONE CSV, and produces "
                             "benchmark_noise_robustness.pdf (thesis Fig 6.4) "
                             "in a single command. Implies --camera; combine "
                             "with --noise-grid to keep the sweep fast "
                             "(default grid otherwise applies per level).")
    parser.add_argument("--camera-freeze", action="store_true",
                        help="Perception figée après la première capture (dead-reckoning ensuite).")
    args = parser.parse_args()

    if args.noise > 0 and args.camera:
        print("[benchmark] ERROR: --noise and --camera are mutually exclusive "
              "(see module docstring). Pick one.")
        sys.exit(1)
    if args.camera and CameraModel is None:
        print("[benchmark] ERROR: --camera requires camera_model.py in the same folder.")
        sys.exit(1)
    if (args.no_lateral_centering or args.no_direction_filter) and (args.camera or args.noise > 0):
        print("[benchmark] NOTE: combining a controller ablation with --camera/--noise mixes "
              "two independent effects (perception vs. controller). The thesis-standard "
              "ablation runs on ground-truth Drake position (no --camera/--noise) so the "
              "comparison isolates the controller component. Continuing anyway.")
    if (args.no_lateral_centering or args.no_direction_filter) and not args.ablation_grid \
            and not args.quick:
        print("[benchmark] NOTE: ablation flag(s) set without --ablation-grid or --quick — "
              "this will run the FULL 36-run/shape grid per condition. Add --ablation-grid "
              "to restrict to the small, targeted 4-run grid instead.")

    out_dir = args.out
    if args.camera_noise_sweep:
        out_dir = os.path.join(out_dir, "camera_noise_sweep")
    elif args.noise > 0:
        out_dir = os.path.join(out_dir, f"noise_{int(args.noise*1000)}mm")
    elif args.camera:
        out_dir = os.path.join(
            out_dir,
            f"camera_hz{int(args.camera_hz)}_lat{int(args.camera_latency*1000)}ms_"
            f"noise{int(args.camera_noise*1000)}mm")

    ablation_tag = ""
    if args.no_lateral_centering:
        ablation_tag += "_noLat"
    if args.no_direction_filter:
        ablation_tag += "_noFilt"
    if ablation_tag:
        out_dir = os.path.join(out_dir, f"ablation{ablation_tag}")
    elif args.ablation_grid:
        out_dir = os.path.join(out_dir, "ablation_baseline")

    os.makedirs(out_dir, exist_ok=True)

    if args.analyse:
        analyse(args.analyse, out_dir)
    elif args.camera_noise_sweep:
        levels = [float(x) for x in args.camera_noise_sweep.split(",")]
        grid = GRID_NOISE if args.noise_grid else (GRID_QUICK if args.quick else GRID_FULL)
        all_rows = []
        merged_csv = os.path.join(out_dir, "benchmark_raw.csv")

        _install_ablation_patch(args.no_lateral_centering, args.no_direction_filter)
        try:
            for sigma in levels:
                print(f"\n[benchmark] === camera noise sweep: sigma = {sigma*1000:.1f} mm "
                      f"({levels.index(sigma)+1}/{len(levels)}) ===")
                camera_kwargs = dict(hz=args.camera_hz, latency_s=args.camera_latency,
                                    noise_std_m=args.camera_noise,
                                    occlusion_prob_during_contact=args.camera_occlusion,
                                    freeze_after_first_capture=args.camera_freeze)
                level_rows = run_benchmark(grid, out_dir,
                                     noise_std=0.0,
                                     shape_filter=args.shape,
                                     camera_kwargs=camera_kwargs,
                                     no_lateral_centering=args.no_lateral_centering,
                                     no_direction_filter=args.no_direction_filter)
                all_rows.extend(level_rows)

                # Re-merge after every level (not just at the end) so an
                # interrupted sweep still leaves a valid, complete-so-far
                # CSV on disk — run_benchmark() truncates benchmark_raw.csv
                # at the start of each call, so without this the previous
                # levels' rows would only survive in memory.
                with open(merged_csv, "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
                    w.writeheader()
                    w.writerows(all_rows)
        finally:
            _remove_ablation_patch()

        print(f"\n[benchmark] merged CSV ({len(all_rows)} runs across {len(levels)} "
              f"noise levels) -> {merged_csv}")
        rows = all_rows
        print("\n[benchmark] Generating figures:")
        plot_success_heatmap(rows, out_dir)
        plot_final_distance(rows, out_dir)
        plot_completion_time(rows, out_dir)
        plot_drake_contact_pct(rows, out_dir)
        plot_noise_robustness(rows, out_dir)
        write_latex_table_condensed(rows, out_dir)
        write_position_table(rows, out_dir)
        write_latex_table(rows, out_dir)
        write_latex_table_ablation(rows, out_dir)
        print_summary(rows)
        print(f"\n[benchmark] Done — {len(rows)} runs — {out_dir}/")
    else:
        grid = GRID_ABLATION if args.ablation_grid else (GRID_QUICK if args.quick else GRID_FULL)
        camera_kwargs = None
        if args.camera:
            camera_kwargs = dict(hz=args.camera_hz, latency_s=args.camera_latency,
                                noise_std_m=args.camera_noise,
                                occlusion_prob_during_contact=args.camera_occlusion,
                                freeze_after_first_capture=args.camera_freeze)

        _install_ablation_patch(args.no_lateral_centering, args.no_direction_filter)
        try:
            rows = run_benchmark(grid, out_dir,
                                 noise_std=args.noise,
                                 shape_filter=args.shape,
                                 camera_kwargs=camera_kwargs,
                                 no_lateral_centering=args.no_lateral_centering,
                                 no_direction_filter=args.no_direction_filter)
        finally:
            _remove_ablation_patch()
        print("\n[benchmark] Generating figures:")
        plot_success_heatmap(rows, out_dir)
        plot_final_distance(rows, out_dir)
        plot_completion_time(rows, out_dir)
        plot_drake_contact_pct(rows, out_dir)
        plot_noise_robustness(rows, out_dir)
        write_latex_table_condensed(rows, out_dir)
        write_position_table(rows, out_dir)
        write_latex_table(rows, out_dir, noise_std=args.noise)
        write_latex_table_ablation(rows, out_dir)
        print_summary(rows)
        print(f"\n[benchmark] Done — {len(rows)} runs — {out_dir}/")
