import gzip
import csv
import sys
import glob
import os
import re


_GAUSS_SKIP_KEYS = {"a3", "m"}
_LIKELIHOOD_NAMES = {
    "linear_clamped": "Linear",
    "negative_log_clamped": "Negative Exponential",
    "gauss_clamped": "Gaussian",
}


def _parse_folder_name(folder):
    parts = re.split(r"[\\/]", folder.rstrip("/\\"))
    start = next((i for i, p in enumerate(parts) if p.startswith("n_robots__")), 0)
    parts = parts[start:]

    is_gauss = "gauss_clamped" in parts
    negate_constants = "linear_clamped" in parts or "negative_log_clamped" in parts
    tokens = []
    for part in parts:
        m = re.fullmatch(r"n_robots__(\d+)", part)
        if m:
            tokens.append(f"R={m.group(1)}")
            continue
        # key__val: a_1__1_0 -> a1=1.0, s__0_3 -> s=0.3
        m = re.fullmatch(r"(.+?)__(\d+)_(\d+)", part)
        if m:
            key = m.group(1).replace("_", "")
            if is_gauss and key in _GAUSS_SKIP_KEYS:
                continue
            sign = "-" if negate_constants else ""
            tokens.append(f"{key}={sign}{m.group(2)}.{m.group(3)}")
            continue
        # likelihood function name
        tokens.append(_LIKELIHOOD_NAMES.get(part, part.replace("_", " ").title()))

    return ", ".join(tokens)


def correct_id_ratios(folder):
    name = _parse_folder_name(folder)
    ratios = []
    for path in sorted(glob.glob(os.path.join(folder, "run_*.csv.gz"))):
        total = 0
        correct = 0
        with gzip.open(path, "rt") as f:
            for row in csv.DictReader(f):
                if not row["posterior_0"]:
                    continue
                total += 1
                if row["robot"] == row["identify_as"]:
                    correct += 1
        ratios.append(correct / total if total else None)
    return name, ratios


def euclidean_errors(folder):
    name = _parse_folder_name(folder)
    errors = []
    for path in sorted(glob.glob(os.path.join(folder, "run_*.csv.gz"))):
        with gzip.open(path, "rt") as f:
            for row in csv.DictReader(f):
                if not row["posterior_0"]:
                    continue
                if not row["real_waldo_pos_0"] or not row["sunburstRobotCalc/waldoPosition_0"]:
                    continue
                wx, wy = float(row["real_waldo_pos_0"]), float(row["real_waldo_pos_1"])
                ex, ey = float(row["sunburstRobotCalc/waldoPosition_0"]), float(row["sunburstRobotCalc/waldoPosition_1"])
                errors.append(((wx - ex) ** 2 + (wy - ey) ** 2) ** 0.5)
    return name, errors


def _last_constant(name):
    vals = re.findall(r"=(-?\d+\.\d+)", name)
    return float(vals[-1]) if vals else 0.0


def all_correct_id_ratios(n_robots_folder):
    results = []
    for folder in sorted(glob.glob(os.path.join(n_robots_folder, "**"), recursive=True)):
        if glob.glob(os.path.join(folder, "run_*.csv.gz")):
            results.append(correct_id_ratios(folder))
    _fn_order = {"Linear": 0, "Negative Exponential": 1, "Gaussian": 2}
    results.sort(key=lambda x: (_fn_order.get(x[0].split(", ")[1], 99), _last_constant(x[0])))
    return results


def _all_experiments(n_robots_folder, metric_fn):
    results = []
    for folder in sorted(glob.glob(os.path.join(n_robots_folder, "**"), recursive=True)):
        if glob.glob(os.path.join(folder, "run_*.csv.gz")):
            results.append(metric_fn(folder))
    _fn_order = {"Linear": 0, "Negative Exponential": 1, "Gaussian": 2}
    results.sort(key=lambda x: (_fn_order.get(x[0].split(", ")[1], 99), _last_constant(x[0])))
    return results


def _all_robots(experiment_folder, metric_fn):
    results = {}
    for folder in sorted(glob.glob(os.path.join(experiment_folder, "n_robots__*"))):
        n_robots = int(re.search(r"n_robots__(\d+)", folder).group(1))
        results[n_robots] = _all_experiments(folder, metric_fn)
    return results


def all_robots_correct_id_ratios(experiment_folder):
    return _all_robots(experiment_folder, correct_id_ratios)


def all_robots_euclidean_errors(experiment_folder):
    return _all_robots(experiment_folder, euclidean_errors)


def _make_boxplot(ax, all_data, ylabel, first_group, color_sets, fn_names, group_size, subgroup_size):
    from matplotlib.patches import Patch

    labels = []
    data = []
    for n_robots, entries in sorted(all_data.items()):
        for name, values in entries:
            labels.append(name)
            data.append([v for v in values if v is not None])

    const_labels = [re.sub(r"^R=\d+,\s*[^,]+,\s*", "", l) for l in labels]
    first_group[:] = const_labels[:group_size]

    bp = ax.boxplot(data, tick_labels=[""] * len(data), patch_artist=True)
    for i, patch in enumerate(bp["boxes"]):
        fi = (i % group_size) // subgroup_size
        pos = i % subgroup_size
        patch.set_facecolor(color_sets[fi][pos])

    ax.set_ylabel(ylabel, fontsize=10, fontweight="bold")
    xform = ax.get_xaxis_transform()

    for gi, n_robots in enumerate(sorted(all_data.keys())):
        group_start = gi * group_size
        center = group_start + group_size / 2 + 0.5
        ax.text(center, -0.16, f"{n_robots} Robots", ha="center", va="top",
                fontweight="bold", fontsize=10, transform=xform)
        if gi > 0:
            ax.axvline(x=group_start + 0.5, color="gray", linestyle="--", linewidth=0.8)
        for fi, fn in enumerate(fn_names):
            sub_start = group_start + fi * subgroup_size
            sub_center = sub_start + subgroup_size / 2 + 0.5
            fn_display = fn.replace(" ", "\n") if fn == "Negative Exponential" else fn
            ax.text(sub_center, -0.05, fn_display, ha="center", va="top",
                    fontstyle="italic", fontsize=10, color=color_sets[fi][2], transform=xform)
            if fi > 0:
                ax.axvline(x=sub_start + 0.5, color="lightgray", linestyle=":", linewidth=0.6)

    legend_handles = []
    legend_labels_list = []
    for fi, fn in enumerate(fn_names):
        legend_handles.append(Patch(visible=False))
        legend_labels_list.append(fn.replace("Negative Exponential", "Negative\nExponential"))
        for pos in range(subgroup_size):
            legend_handles.append(Patch(facecolor=color_sets[fi][pos]))
            legend_labels_list.append(first_group[fi * subgroup_size + pos])
    ax.legend(legend_handles, legend_labels_list, loc="lower left", ncols=3,
              prop={"size": 12, "weight": "bold"})
    ax.set_xlim(0.3, len(data) + 0.7)


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "font.weight": "bold"})

    default = "./dars26/Likelihood Functions"
    experiment_folder = sys.argv[1] if len(sys.argv) > 1 else default

    fn_names = ["Linear", "Negative Exponential", "Gaussian"]
    group_size = 9
    subgroup_size = 3
    color_sets = [
        ["#BDD7EE", "#5B9BD5", "#1F4E79"],
        ["#FCE4C8", "#F4B183", "#C55A11"],
        ["#C6E0B4", "#70AD47", "#375623"],
    ]
    first_group = []

    fig1, ax1 = plt.subplots(figsize=(14, 4.5))
    _make_boxplot(ax1, all_robots_correct_id_ratios(experiment_folder),
                  "Correct ID Assignment Rate", first_group,
                  color_sets, fn_names, group_size, subgroup_size)
    ax1.set_ylim(0, 1)
    plt.tight_layout()
    plt.savefig("boxplot_likelihood_functions.png", dpi=150, bbox_inches="tight")
    print("Saved boxplot_likelihood_functions.png")