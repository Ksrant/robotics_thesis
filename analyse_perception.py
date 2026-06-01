"""
analyse_perception.py
═══════════════════════════════════════════════════════════════════════════════
Script d'analyse post-run de la perception.

UTILISATION
───────────
    python analyse_perception.py                        # lit figures/perception_eval.npz
    python analyse_perception.py mon_run.npz            # fichier custom
    python analyse_perception.py --multi run1.npz run2.npz run3.npz

SORTIES (dans figures/)
───────────────────────
    perception_error_timeline.pdf   Erreur Kalman vs temps, colorée par phase FSM
    perception_error_hist.pdf       Distribution de l'erreur (total + par phase)
    perception_occlusion.pdf        Analyse des séquences d'occlusion
    perception_gt_vs_est.pdf        Trajectoire GT vs estimation (vue de dessus)
    perception_metrics.tex          Tableau LaTeX prêt à coller dans le mémoire
    perception_summary.txt          Résumé console
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D

# ─── config plot ─────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family":       "serif",
    "font.size":         11,
    "axes.titlesize":    12,
    "axes.labelsize":    11,
    "legend.fontsize":   9,
    "xtick.labelsize":   9,
    "ytick.labelsize":   9,
    "figure.dpi":        150,
    "savefig.dpi":       300,
    "savefig.bbox":      "tight",
    "axes.grid":         True,
    "grid.alpha":        0.35,
    "axes.spines.top":   False,
    "axes.spines.right": False,
})

FSM_NAMES  = {0: "APPROACH", 1: "PUSH", 2: "REPOSITION", 3: "DONE", -1: "?"}
FSM_COLORS = {0: "#4C72B0", 1: "#DD8452", 2: "#55A868", 3: "#C44E52", -1: "#8C8C8C"}
OUT_DIR    = "figures"


# ═══════════════════════════════════════════════════════════════════════════════
# CHARGEMENT
# ═══════════════════════════════════════════════════════════════════════════════

def load(path: str) -> dict:
    d = np.load(path)
    out = {k: d[k] for k in d.files}
    # Assure que error est en mm pour l'affichage
    out["error_mm"] = out["error"] * 1000.0
    # Segments par phase FSM
    out["phases"] = _split_phases(out["t"], out["fsm"])
    return out


def _split_phases(t, fsm):
    """Retourne liste de dict {fsm_id, t_start, t_end, mask}."""
    phases = []
    if len(fsm) == 0:
        return phases
    cur = fsm[0]
    start_i = 0
    for i in range(1, len(fsm)):
        if fsm[i] != cur or i == len(fsm) - 1:
            end_i = i if fsm[i] != cur else i + 1
            phases.append(dict(
                fsm_id  = int(cur),
                t_start = float(t[start_i]),
                t_end   = float(t[min(end_i, len(t)-1)]),
                mask    = slice(start_i, end_i),
            ))
            cur     = fsm[i]
            start_i = i
    return phases


# ═══════════════════════════════════════════════════════════════════════════════
# MÉTRIQUES
# ═══════════════════════════════════════════════════════════════════════════════

def compute_metrics(d: dict) -> dict:
    err_mm  = d["error_mm"]
    fsm     = d["fsm"]
    det     = d["detected"]
    n_consec = d.get("n_consec", np.zeros_like(err_mm, dtype=int))
    t       = d["t"]

    m = {}
    m["n_frames_total"]   = len(err_mm)
    m["duration_s"]       = float(t[-1] - t[0]) if len(t) > 1 else 0.0

    # Erreur globale
    m["err_mean_mm"]  = float(np.nanmean(err_mm))
    m["err_median_mm"]= float(np.nanmedian(err_mm))
    m["err_p95_mm"]   = float(np.nanpercentile(err_mm, 95))
    m["err_max_mm"]   = float(np.nanmax(err_mm))
    m["err_std_mm"]   = float(np.nanstd(err_mm))

    # Par phase FSM
    m["by_phase"] = {}
    for fid in [0, 1, 2, 3]:
        mask = fsm == fid
        if mask.sum() == 0:
            continue
        e = err_mm[mask]
        m["by_phase"][fid] = dict(
            n       = int(mask.sum()),
            mean_mm = float(np.nanmean(e)),
            p95_mm  = float(np.nanpercentile(e, 95)),
            max_mm  = float(np.nanmax(e)),
        )

    # Occlusions : séquences consécutives de n_consec > 0
    occ_runs = _occlusion_runs(n_consec)
    m["n_occ_events"]    = len(occ_runs)
    m["occ_total_frames"]= int(sum(r["length"] for r in occ_runs))
    m["occ_rate_pct"]    = 100.0 * m["occ_total_frames"] / max(1, m["n_frames_total"])
    if occ_runs:
        lengths = [r["length"] for r in occ_runs]
        m["occ_mean_len"]  = float(np.mean(lengths))
        m["occ_max_len"]   = float(np.max(lengths))
    else:
        m["occ_mean_len"]  = 0.0
        m["occ_max_len"]   = 0.0

    m["occ_runs"] = occ_runs
    return m


def _occlusion_runs(n_consec):
    """Identifie les runs contigus de frames manquées."""
    runs = []
    in_run = False
    start = 0
    for i, v in enumerate(n_consec):
        if v > 0 and not in_run:
            in_run = True
            start  = i
        elif v == 0 and in_run:
            runs.append(dict(start=start, end=i-1, length=i-start))
            in_run = False
    if in_run:
        runs.append(dict(start=start, end=len(n_consec)-1,
                         length=len(n_consec)-start))
    return runs


# ═══════════════════════════════════════════════════════════════════════════════
# PLOTS
# ═══════════════════════════════════════════════════════════════════════════════

def _shade_phases(ax, phases):
    """Colorie le fond de l'axe selon la phase FSM."""
    for ph in phases:
        ax.axvspan(ph["t_start"], ph["t_end"],
                   color=FSM_COLORS[ph["fsm_id"]], alpha=0.12, linewidth=0)


def _fsm_legend():
    return [mpatches.Patch(color=FSM_COLORS[i], alpha=0.4, label=FSM_NAMES[i])
            for i in [0, 1, 2, 3] if i in FSM_NAMES]


# ── 1. Timeline de l'erreur ───────────────────────────────────────────────────

def plot_error_timeline(d, m, out_dir):
    t       = d["t"]
    err_mm  = d["error_mm"]
    phases  = d["phases"]
    n_consec = d.get("n_consec", np.zeros(len(t), dtype=int))

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                              gridspec_kw={"height_ratios": [3, 1]})

    # Axe principal : erreur
    ax = axes[0]
    _shade_phases(ax, phases)
    ax.plot(t, err_mm, color="#2c7bb6", lw=0.8, alpha=0.85, label="Erreur Kalman ‖p̂−p*‖")
    # Fenêtre glissante 1 s
    win = max(1, int(30))
    if len(err_mm) >= win:
        smooth = np.convolve(err_mm, np.ones(win)/win, mode="same")
        ax.plot(t, smooth, color="#d7191c", lw=1.5, label=f"Moyenne glissante ({win} frames)")
    ax.axhline(m["err_mean_mm"],  ls="--", color="gray",  lw=1, label=f"Moyenne = {m['err_mean_mm']:.1f} mm")
    ax.axhline(m["err_p95_mm"],   ls=":",  color="orange", lw=1, label=f"P95 = {m['err_p95_mm']:.1f} mm")
    ax.set_ylabel("Erreur [mm]")
    ax.set_title("Erreur de localisation du cube — estimation Kalman vs vérité terrain Drake")
    handles = ax.get_legend_handles_labels()[0] + _fsm_legend()
    labels  = ax.get_legend_handles_labels()[1] + [FSM_NAMES[i] for i in [0,1,2,3] if i in FSM_NAMES]
    ax.legend(handles, labels, loc="upper right", ncol=2)

    # Axe bas : détection / occlusion
    ax2 = axes[1]
    _shade_phases(ax2, phases)
    occ_binary = (n_consec > 0).astype(float)
    ax2.fill_between(t, occ_binary, step="post", color="#e74c3c", alpha=0.7, label="Occlusion")
    ax2.fill_between(t, 1 - occ_binary, step="post", color="#2ecc71", alpha=0.5, label="Détection OK")
    ax2.set_ylabel("Détection")
    ax2.set_yticks([0, 1])
    ax2.set_yticklabels(["Non", "Oui"])
    ax2.set_xlabel("Temps [s]")
    ax2.legend(loc="upper right")

    plt.tight_layout()
    path = os.path.join(out_dir, "perception_error_timeline.pdf")
    plt.savefig(path); plt.close()
    print(f"  → {path}")


# ── 2. Distribution de l'erreur par phase ────────────────────────────────────

def plot_error_histogram(d, m, out_dir):
    err_mm = d["error_mm"]
    fsm    = d["fsm"]

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    # Global
    ax = axes[0]
    ax.hist(err_mm[~np.isnan(err_mm)], bins=60, color="#2c7bb6",
            edgecolor="white", linewidth=0.3, density=True)
    ax.axvline(m["err_mean_mm"],   color="red",    lw=1.5, ls="--",
               label=f"Moyenne {m['err_mean_mm']:.1f} mm")
    ax.axvline(m["err_p95_mm"],    color="orange", lw=1.5, ls=":",
               label=f"P95 {m['err_p95_mm']:.1f} mm")
    ax.set_xlabel("Erreur [mm]"); ax.set_ylabel("Densité")
    ax.set_title("Distribution globale de l'erreur")
    ax.legend()

    # Par phase
    ax = axes[1]
    for fid, color in [(0,"#4C72B0"),(1,"#DD8452"),(2,"#55A868"),(3,"#C44E52")]:
        mask = fsm == fid
        if mask.sum() < 5:
            continue
        e = err_mm[mask]
        ax.hist(e[~np.isnan(e)], bins=40, color=color, alpha=0.6,
                density=True, label=FSM_NAMES[fid])
    ax.set_xlabel("Erreur [mm]"); ax.set_ylabel("Densité")
    ax.set_title("Distribution par phase FSM")
    ax.legend()

    plt.tight_layout()
    path = os.path.join(out_dir, "perception_error_hist.pdf")
    plt.savefig(path); plt.close()
    print(f"  → {path}")


# ── 3. Trajectoire GT vs estimation (vue de dessus) ──────────────────────────

def plot_trajectories(d, m, out_dir):
    p_est = d["p_est"]
    p_gt  = d["p_gt"]
    fsm   = d["fsm"]
    t     = d["t"]

    fig, ax = plt.subplots(figsize=(7, 6))

    # Trajectoire GT colorée par phase
    for fid in [0, 1, 2, 3]:
        mask = fsm == fid
        if mask.sum() < 2:
            continue
        idx = np.where(mask)[0]
        ax.plot(p_gt[idx, 0], p_gt[idx, 1],
                color=FSM_COLORS[fid], lw=2.0, alpha=0.9,
                label=f"GT — {FSM_NAMES[fid]}")

    # Trajectoire estimée (ligne fine)
    ax.plot(p_est[:, 0], p_est[:, 1],
            color="black", lw=0.7, alpha=0.5, ls="--", label="Estimation Kalman")

    # Erreurs (segments verticaux tous les N points)
    step = max(1, len(t) // 80)
    for i in range(0, len(t), step):
        ax.plot([p_est[i,0], p_gt[i,0]], [p_est[i,1], p_gt[i,1]],
                color="red", lw=0.4, alpha=0.4)

    # Positions initiale et finale
    ax.scatter(*p_gt[0],  marker="s", s=80, color="green",  zorder=5, label="Départ (GT)")
    ax.scatter(*p_gt[-1], marker="*", s=120, color="purple", zorder=5, label="Arrivée (GT)")

    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Trajectoire du cube — vérité terrain vs estimation Kalman")
    ax.legend(fontsize=8, ncol=2)
    ax.set_aspect("equal")

    plt.tight_layout()
    path = os.path.join(out_dir, "perception_gt_vs_est.pdf")
    plt.savefig(path); plt.close()
    print(f"  → {path}")


# ── 4. Analyse des occlusions ────────────────────────────────────────────────

def plot_occlusions(d, m, out_dir):
    n_consec = d.get("n_consec", np.zeros(len(d["t"]), dtype=int))
    t        = d["t"]
    phases   = d["phases"]
    runs     = m["occ_runs"]

    fig, axes = plt.subplots(2, 1, figsize=(10, 5), sharex=False)

    # Timeline du compteur consécutif
    ax = axes[0]
    _shade_phases(ax, phases)
    ax.plot(t, n_consec, color="#e74c3c", lw=0.9)
    ax.fill_between(t, n_consec, alpha=0.3, color="#e74c3c")
    ax.axhline(4,  ls=":", color="orange", lw=1, label="Seuil amortissement (4)")
    ax.axhline(10, ls=":", color="red",    lw=1, label="Seuil gel Kalman (10)")
    ax.set_ylabel("Frames consécutives\nsans détection")
    ax.set_xlabel("Temps [s]")
    ax.set_title(f"Séquences d'occlusion — {m['n_occ_events']} événements, "
                 f"{m['occ_rate_pct']:.1f}% du temps")
    handles = ax.get_legend_handles_labels()[0] + _fsm_legend()
    labels  = ax.get_legend_handles_labels()[1] + [FSM_NAMES[i] for i in [0,1,2,3]]
    ax.legend(handles, labels, loc="upper right", ncol=3, fontsize=8)

    # Histogramme des durées d'occlusion
    ax2 = axes[1]
    if runs:
        lengths = [r["length"] for r in runs]
        ax2.hist(lengths, bins=range(1, max(lengths)+2), color="#e74c3c",
                 edgecolor="white", linewidth=0.4)
        ax2.axvline(4,  ls=":", color="orange", lw=1.2)
        ax2.axvline(10, ls=":", color="red",    lw=1.2)
        ax2.set_xlabel("Durée de l'occlusion [frames]")
        ax2.set_ylabel("Nombre d'événements")
        ax2.set_title(f"Distribution des durées — moyenne {m['occ_mean_len']:.1f} frames, "
                      f"max {m['occ_max_len']:.0f} frames")
    else:
        ax2.text(0.5, 0.5, "Aucune occlusion détectée", ha="center", va="center",
                 transform=ax2.transAxes, fontsize=13, color="green")

    plt.tight_layout()
    path = os.path.join(out_dir, "perception_occlusion.pdf")
    plt.savefig(path); plt.close()
    print(f"  → {path}")


# ── 5. Erreur par phase — boîtes à moustaches ────────────────────────────────

def plot_boxplot_by_phase(d, m, out_dir):
    err_mm = d["error_mm"]
    fsm    = d["fsm"]

    data, labels, colors = [], [], []
    for fid in [0, 1, 2, 3]:
        mask = fsm == fid
        if mask.sum() < 5:
            continue
        e = err_mm[mask]
        data.append(e[~np.isnan(e)])
        labels.append(FSM_NAMES[fid])
        colors.append(FSM_COLORS[fid])

    if not data:
        return

    fig, ax = plt.subplots(figsize=(6, 4))
    bp = ax.boxplot(data, patch_artist=True, notch=False,
                    medianprops=dict(color="black", lw=2))
    for patch, color in zip(bp["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Erreur [mm]")
    ax.set_title("Distribution de l'erreur de localisation par phase FSM")

    plt.tight_layout()
    path = os.path.join(out_dir, "perception_boxplot.pdf")
    plt.savefig(path); plt.close()
    print(f"  → {path}")


# ═══════════════════════════════════════════════════════════════════════════════
# TABLEAU LaTeX
# ═══════════════════════════════════════════════════════════════════════════════

def write_latex_table(m: dict, out_dir: str):
    lines = [
        r"\begin{table}[ht]",
        r"  \centering",
        r"  \caption{Métriques quantitatives de la perception (simulation Drake)}",
        r"  \label{tab:perception_metrics}",
        r"  \begin{tabular}{lrr}",
        r"    \toprule",
        r"    Métrique & Valeur & Unité \\",
        r"    \midrule",
        r"    \multicolumn{3}{l}{\textit{Erreur de localisation — global}} \\",
        f"    Erreur moyenne           & {m['err_mean_mm']:.2f}  & mm \\\\",
        f"    Erreur médiane           & {m['err_median_mm']:.2f} & mm \\\\",
        f"    Erreur P95               & {m['err_p95_mm']:.2f}   & mm \\\\",
        f"    Erreur maximale          & {m['err_max_mm']:.2f}   & mm \\\\",
        f"    Écart-type               & {m['err_std_mm']:.2f}   & mm \\\\",
        r"    \midrule",
        r"    \multicolumn{3}{l}{\textit{Par phase FSM}} \\",
    ]

    for fid, name in [(0,"APPROACH"),(1,"PUSH"),(2,"REPOSITION")]:
        if fid not in m["by_phase"]:
            continue
        ph = m["by_phase"][fid]
        lines += [
            f"    {name} — moyenne & {ph['mean_mm']:.2f} & mm \\\\",
            f"    {name} — P95     & {ph['p95_mm']:.2f}  & mm \\\\",
        ]

    lines += [
        r"    \midrule",
        r"    \multicolumn{3}{l}{\textit{Occlusions}} \\",
        f"    Nombre d'événements      & {m['n_occ_events']}     & — \\\\",
        f"    Taux d'occlusion         & {m['occ_rate_pct']:.1f} & \\% \\\\",
        f"    Durée moyenne            & {m['occ_mean_len']:.1f} & frames \\\\",
        f"    Durée maximale           & {m['occ_max_len']:.0f}  & frames \\\\",
        r"    \bottomrule",
        r"  \end{tabular}",
        r"\end{table}",
    ]

    path = os.path.join(out_dir, "perception_metrics.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  → {path}")


# ═══════════════════════════════════════════════════════════════════════════════
# RÉSUMÉ CONSOLE
# ═══════════════════════════════════════════════════════════════════════════════

def print_summary(m: dict):
    print("\n" + "═"*60)
    print("  RÉSUMÉ — ÉVALUATION PERCEPTION")
    print("═"*60)
    print(f"  Durée simulée         : {m['duration_s']:.1f} s")
    print(f"  Frames analysées      : {m['n_frames_total']}")
    print()
    print(f"  Erreur moyenne        : {m['err_mean_mm']:.2f} mm")
    print(f"  Erreur médiane        : {m['err_median_mm']:.2f} mm")
    print(f"  Erreur P95            : {m['err_p95_mm']:.2f} mm")
    print(f"  Erreur max            : {m['err_max_mm']:.2f} mm")
    print()
    print(f"  Phase PUSH — moy      : {m['by_phase'].get(1, {}).get('mean_mm', 0):.2f} mm")
    print(f"  Phase PUSH — P95      : {m['by_phase'].get(1, {}).get('p95_mm',  0):.2f} mm")
    print()
    print(f"  Événements occlusion  : {m['n_occ_events']}")
    print(f"  Taux d'occlusion      : {m['occ_rate_pct']:.1f} %")
    print(f"  Durée moy. occlusion  : {m['occ_mean_len']:.1f} frames")
    print(f"  Durée max. occlusion  : {m['occ_max_len']:.0f} frames")
    print("═"*60 + "\n")


# ═══════════════════════════════════════════════════════════════════════════════
# POINT D'ENTRÉE
# ═══════════════════════════════════════════════════════════════════════════════

def analyse_single(npz_path: str, out_dir: str = OUT_DIR):
    print(f"\n[analyse_perception] Chargement : {npz_path}")
    d = load(npz_path)
    m = compute_metrics(d)

    os.makedirs(out_dir, exist_ok=True)
    print("[analyse_perception] Génération des figures :")
    plot_error_timeline(d, m, out_dir)
    plot_error_histogram(d, m, out_dir)
    plot_trajectories(d, m, out_dir)
    plot_occlusions(d, m, out_dir)
    plot_boxplot_by_phase(d, m, out_dir)
    write_latex_table(m, out_dir)
    print_summary(m)
    return m


def analyse_multi(paths: list, out_dir: str = OUT_DIR):
    """
    Compare plusieurs runs (masses différentes, positions initiales...).
    Génère un graphe d'erreur superposé et un tableau comparatif.
    """
    os.makedirs(out_dir, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    summary_rows = []

    palette = plt.cm.tab10.colors
    for i, path in enumerate(paths):
        label = os.path.splitext(os.path.basename(path))[0]
        d = load(path)
        m = compute_metrics(d)
        color = palette[i % len(palette)]

        # Timeline
        axes[0].plot(d["t"], d["error_mm"], lw=0.6, alpha=0.6, color=color)
        win = max(1, 30)
        if len(d["error_mm"]) >= win:
            sm = np.convolve(d["error_mm"], np.ones(win)/win, mode="same")
            axes[0].plot(d["t"], sm, lw=1.8, color=color, label=label)

        # Boxplot data
        summary_rows.append((label, m["err_mean_mm"], m["err_p95_mm"],
                              m["occ_rate_pct"], m["n_occ_events"]))

    axes[0].set_xlabel("Temps [s]"); axes[0].set_ylabel("Erreur [mm]")
    axes[0].set_title("Erreur Kalman — comparaison multi-runs")
    axes[0].legend(fontsize=8)

    # Barplot moyenne par run
    labels_r = [r[0] for r in summary_rows]
    means    = [r[1] for r in summary_rows]
    p95s     = [r[2] for r in summary_rows]
    x        = np.arange(len(labels_r))
    axes[1].bar(x - 0.2, means, 0.35, label="Moyenne", color="#2c7bb6")
    axes[1].bar(x + 0.2, p95s,  0.35, label="P95",     color="#d7191c", alpha=0.7)
    axes[1].set_xticks(x); axes[1].set_xticklabels(labels_r, rotation=20, ha="right")
    axes[1].set_ylabel("Erreur [mm]"); axes[1].legend()
    axes[1].set_title("Résumé par run")

    plt.tight_layout()
    path_out = os.path.join(out_dir, "perception_multi_run.pdf")
    plt.savefig(path_out); plt.close()
    print(f"  → {path_out}")

    # Tableau LaTeX comparatif
    tex_lines = [
        r"\begin{table}[ht]",
        r"  \centering",
        r"  \caption{Comparaison des runs — métriques de perception}",
        r"  \label{tab:perception_multi}",
        r"  \begin{tabular}{lrrrr}",
        r"    \toprule",
        r"    Run & Moy. [mm] & P95 [mm] & Occ. [\%] & Nb. occ. \\",
        r"    \midrule",
    ]
    for row in summary_rows:
        tex_lines.append(
            f"    {row[0]} & {row[1]:.2f} & {row[2]:.2f} & {row[3]:.1f} & {row[4]} \\\\"
        )
    tex_lines += [r"    \bottomrule", r"  \end{tabular}", r"\end{table}"]
    tex_path = os.path.join(out_dir, "perception_metrics_multi.tex")
    with open(tex_path, "w") as f:
        f.write("\n".join(tex_lines) + "\n")
    print(f"  → {tex_path}")


# ─── CLI ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyse post-run de la perception")
    parser.add_argument("files", nargs="*",
                        default=["figures/perception_eval.npz"],
                        help="Fichier(s) .npz à analyser")
    parser.add_argument("--multi", action="store_true",
                        help="Mode comparaison multi-runs")
    parser.add_argument("--out", default=OUT_DIR, help="Dossier de sortie")
    args = parser.parse_args()

    if args.multi or len(args.files) > 1:
        analyse_multi(args.files, args.out)
    else:
        analyse_single(args.files[0], args.out)
