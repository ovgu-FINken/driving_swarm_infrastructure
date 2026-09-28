import pandas as pd
import numpy as np


# --- what to run on / where to save ---
# glob for the parameter-combo folders holding run_*.csv.gz (three param levels deep)
INPUT_GLOB = "dars26/Hyperparameters/n_robots__5/*/*/*"
OUTPUT_FILE = "boxplot_hyperparameters.png"  # figure written here


def correct_id_ratio(run_file: str) -> float:
    """Fraction of rows in one run where the robot was identified correctly."""
    df = pd.read_csv(run_file)
    sub = df[df["posterior_0"].notna()]
    if len(sub) == 0:
        return float("nan")
    return (sub["robot"] == sub["identify_as"]).sum() / len(sub)


def correct_id_ratios_for_folder(path: str) -> list[float]:
    """One correct-ID ratio per run in the folder."""
    import glob
    run_files = sorted(glob.glob(f"{path}/run_*.csv.gz"))
    return [correct_id_ratio(f) for f in run_files]


def euclidean_errors_for_folder(path: str) -> list[float]:
    """All per-row distances between true and estimated target positions, pooled across runs (meters)."""
    import glob
    run_files = sorted(glob.glob(f"{path}/run_*.csv.gz"))
    values = []
    for f in run_files:
        df = pd.read_csv(f)
        sub = df[df["posterior_0"].notna()].dropna(
            subset=["real_waldo_pos_0", "real_waldo_pos_1",
                    "sunburstRobotCalc/waldoPosition_0", "sunburstRobotCalc/waldoPosition_1"]
        )
        err = np.sqrt(
            (sub["real_waldo_pos_0"] - sub["sunburstRobotCalc/waldoPosition_0"])**2 +
            (sub["real_waldo_pos_1"] - sub["sunburstRobotCalc/waldoPosition_1"])**2
        )
        values.extend(err.tolist())
    return values


_PARAM_NAMES = {
    "n_robots": None,
    "dist_threshold": None,
    "angle_threshold": None,
    "old_error_penalty_scale": None,
}


def folder_label(path: str) -> str:
    """Build a 'name = value, ...' label from the 'name__value' parts in the folder path."""
    parts = path.replace("\\", "/").split("/")
    params = {}
    for part in parts:
        if "__" in part:
            name, value = part.split("__", 1)
            value = value.replace("_", ".")
            params[name] = value
    return ", ".join(
        f"{_PARAM_NAMES.get(k, k)} = {v}"
        for k, v in params.items()
        if _PARAM_NAMES.get(k, k) is not None
    )


if __name__ == "__main__":
    import glob
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "font.weight": "bold"})

    def get_param(path, param):
        """Read a single parameter value out of the folder path by name."""
        for part in path.replace("\\", "/").split("/"):
            if part.startswith(param + "__"):
                return part.split("__", 1)[1].replace("_", ".")
        return None

    PENALTY_COLORS = {"1.02": "#5B9BD5", "1.05": "#F4B183"}

    folders = sorted(glob.glob(INPUT_GLOB))
    labels = [folder_label(f) for f in folders]
    data = [correct_id_ratios_for_folder(f) for f in folders]
    errors = [euclidean_errors_for_folder(f) for f in folders]
    dist_thresholds = [get_param(f, "dist_threshold") for f in folders]
    angle_thresholds = [get_param(f, "angle_threshold") for f in folders]
    penalties = [get_param(f, "old_error_penalty_scale") for f in folders]

    AT_COLORS = ["#1F4E79", "#C55A11", "#375623"]
    AT_LABELS = {"0.08727": "π/36", "0.1309": "π/24", "0.261799": "π/12"}

    OFFSET = 0.2
    WIDTH = 0.35
    centers = list(range(1, len(folders) + 1))
    pos_left  = [c - OFFSET for c in centers]
    pos_right = [c + OFFSET for c in centers]

    fig, ax = plt.subplots(figsize=(14, 5.0))
    bp = ax.boxplot(data, labels=[""] * len(data), patch_artist=True,
                    positions=pos_left, widths=WIDTH)
    for patch, penalty in zip(bp["boxes"], penalties):
        patch.set_facecolor(PENALTY_COLORS.get(penalty, "white"))
    ax.set_ylabel("Correct Assignment Rate", fontsize=10, fontweight="bold")
    ax.set_ylim(0, 1)
    ax.set_xlim(0.5, len(folders) + 0.5)
    ax.set_xticks(centers)
    ax.set_xticklabels([""] * len(centers))

    ax2 = ax.twinx()
    bp2 = ax2.boxplot(errors, labels=[""] * len(errors), patch_artist=True,
                      positions=pos_right, widths=WIDTH)
    for patch, penalty in zip(bp2["boxes"], penalties):
        patch.set_facecolor(PENALTY_COLORS.get(penalty, "white"))
        patch.set_alpha(0.5)
    ax2.set_ylabel("Euclidean Error (m)", fontsize=10, fontweight="bold")
    ax2.yaxis.label.set_rotation(-90)
    ax2.yaxis.labelpad = 15

    # group positions by dist_threshold and (dist_threshold, angle_threshold)
    groups = {}
    at_groups = {}
    for i, (dt, at) in enumerate(zip(dist_thresholds, angle_thresholds)):
        groups.setdefault(dt, []).append(i + 1)
        at_groups.setdefault((dt, at), []).append(i + 1)

    unique_ats = list(dict.fromkeys(angle_thresholds))
    xform = ax.get_xaxis_transform()

    # dark gray dashed separator between dist_threshold groups
    dt_boundaries = set()
    for dt in list(groups.keys())[:-1]:
        sep = groups[dt][-1] + 0.5
        dt_boundaries.add(sep)
        ax.axvline(sep, color="gray", linestyle="--", linewidth=0.8)

    # light gray dotted separators between angle_threshold groups
    for (_, _at), positions in at_groups.items():
        sep = positions[-1] + 0.5
        if sep not in dt_boundaries and sep <= len(folders):
            ax.axvline(sep, color="lightgray", linestyle=":", linewidth=0.6)

    # angle threshold labels — italic, colored, right below x-axis
    for (_, at), positions in at_groups.items():
        mid = (positions[0] + positions[-1]) / 2
        color = AT_COLORS[unique_ats.index(at) % len(AT_COLORS)]
        ax.text(mid, -0.05, f"Angle Threshold\n{AT_LABELS.get(at, at)}", transform=xform,
                ha="center", va="top", fontsize=10, fontstyle="italic", color=color)

    # distance threshold labels — bold, just below angle threshold labels
    for dt, positions in groups.items():
        mid = (positions[0] + positions[-1]) / 2
        ax.text(mid, -0.16, f"Distance Threshold {dt} m", transform=xform,
                ha="center", va="top", fontsize=10, fontweight="bold")

    # legend for penalty colors
    from matplotlib.patches import Patch
    legend_handles = []
    for p, c in PENALTY_COLORS.items():
        legend_handles.append(Patch(facecolor=c, label=f"Stale Penalty = {p} (Assignment Rate)"))
        legend_handles.append(Patch(facecolor=c, alpha=0.5, label=f"Stale Penalty = {p} (Euclidean Error)"))
    ax.legend(handles=legend_handles, loc="upper center", ncols=2,
              bbox_to_anchor=(0.5, -0.20), bbox_transform=ax.transAxes,
              prop={"size": 12, "weight": "bold"})

    plt.tight_layout()
    plt.savefig(OUTPUT_FILE, dpi=150, bbox_inches="tight")
    print(f"Saved {OUTPUT_FILE}")