import pandas as pd


def correct_id_ratio(run_file: str) -> float:
    df = pd.read_csv(run_file)
    sub = df[df["posterior_0"].notna()]
    if len(sub) == 0:
        return float("nan")
    return (sub["robot"] == sub["identify_as"]).sum() / len(sub)


def correct_id_ratios_for_folder(path: str) -> list[float]:
    import glob
    run_files = sorted(glob.glob(f"{path}/run_*.csv.gz"))
    return [correct_id_ratio(f) for f in run_files]


_PARAM_NAMES = {
    "n_robots": None,
    "dist_threshold": None,
    "angle_threshold": None,
    "old_error_penalty_scale": None,
}


def folder_label(path: str) -> str:
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
        for part in path.replace("\\", "/").split("/"):
            if part.startswith(param + "__"):
                return part.split("__", 1)[1].replace("_", ".")
        return None

    PENALTY_COLORS = {"1.02": "#5B9BD5", "1.05": "#F4B183"}

    folders = sorted(glob.glob("dars26/Hyperparameters/n_robots__5/*/*/*"))
    labels = [folder_label(f) for f in folders]
    data = [correct_id_ratios_for_folder(f) for f in folders]
    dist_thresholds = [get_param(f, "dist_threshold") for f in folders]
    angle_thresholds = [get_param(f, "angle_threshold") for f in folders]
    penalties = [get_param(f, "old_error_penalty_scale") for f in folders]

    AT_COLORS = ["#1F4E79", "#C55A11", "#375623"]
    AT_LABELS = {"0.08727": "π/36", "0.1309": "π/24", "0.261799": "π/12"}

    fig, ax = plt.subplots(figsize=(14, 4.5))
    bp = ax.boxplot(data, labels=[""] * len(data), patch_artist=True)
    for patch, penalty in zip(bp["boxes"], penalties):
        patch.set_facecolor(PENALTY_COLORS.get(penalty, "white"))
    ax.set_ylabel("Correct ID Assignment Rate", fontsize=10, fontweight="bold")
    ax.set_ylim(0, 1)

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
    legend_handles = [Patch(facecolor=c, label=f"Stale Penalty = {p}") for p, c in PENALTY_COLORS.items()]
    ax.legend(handles=legend_handles, loc="lower left", prop={"size": 12, "weight": "bold"})

    plt.tight_layout()
    plt.savefig("boxplot_hyperparameters.png", dpi=150, bbox_inches="tight")
    print("Saved boxplot_hyperparameters.png")