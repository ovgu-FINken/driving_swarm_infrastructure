import os
import glob
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import streamlit as st
from collections import defaultdict

# =========================================================
# CONFIG
# =========================================================

st.set_page_config(layout="wide")
st.title("Experiment Explorer")

LIKELIHOOD_SCHEMA = {
    "linear_clamped": ["a_1"],
    "negative_log_clamped": ["a_2"],
    "gauss_clamped": ["a_3", "m", "s"],
}

base_dir = st.text_input(
    "Experiment directory",
    value="./dars26/experiment_2026-04-22_17-13-26/"
)

# =========================================================
# HELPERS
# =========================================================

def parse_params_file(path):
    params = {}
    if not os.path.exists(path):
        return params

    with open(path, "r") as f:
        for line in f:
            if ":" in line:
                k, v = line.strip().split(":", 1)
                params[k.strip()] = v.strip()
    return params

def compute_correct_id_stats(runs):
    """
    Per run + per robot:
    - normalized correctness in [0, 1]
      (robot == identify_as)
    """

    rows = []

    for run_idx, df in enumerate(runs):

        if "robot" not in df.columns or "identify_as" not in df.columns:
            continue

        df = df.copy()

        # correctness as boolean
        df["correct"] = df["robot"] == df["identify_as"]

        for robot, rdf in df.groupby("robot"):

            n_rows = len(rdf)

            if n_rows == 0:
                continue

            # normalized correctness (bounded [0,1])
            correctness = rdf["correct"].mean()

            rows.append({
                "run": run_idx,
                "robot": robot,
                "correctness": float(correctness),  # already normalized
                "rows": n_rows
            })

    return pd.DataFrame(rows)

def build_global_correctness_table(data):
    rows = []

    for n_robots, lf_dict in data.items():
        for lf, exp_dict in lf_dict.items():

            schema_params = LIKELIHOOD_SCHEMA.get(lf, [])

            for exp_path, exp in exp_dict.items():

                runs = exp["runs"]
                params = exp["params"]

                #param_id = f"{lf} | {exp_path.split('/')[-1]}"
                param_id = f"{exp_path.split('/')[-1]}"

                for run_idx, df in enumerate(runs):

                    if "robot" not in df.columns or "identify_as" not in df.columns:
                        continue

                    df = df.copy()
                    df["correctness"] = (df["robot"] == df["identify_as"]).astype(float)

                    for robot, rdf in df.groupby("robot"):

                        rows.append({
                            "n_robots": str(n_robots),
                            "likelihood_function": lf,
                            "param_id": param_id,   # now guaranteed unique
                            "run": run_idx,
                            "robot": robot,
                            "correctness": rdf["correctness"].mean()
                        })

    return pd.DataFrame(rows)

def cut_and_reindex_time(df):
    """
    Hard cut at first global posterior event and
    remove rows without any posterior values.
    """

    df = df.copy()

    # ensure numeric time
    df["t"] = pd.to_numeric(df["t"], errors="coerce")

    posterior_cols = [c for c in df.columns if c.startswith("posterior")]

    if not posterior_cols:
        df["t"] = 0
        return df

    # detect first posterior event
    mask = df[posterior_cols].notna().any(axis=1)

    if not mask.any():
        df["t"] = 0
        return df

    t0 = df.loc[mask, "t"].iloc[0]

    # hard cut
    df = df[df["t"] >= t0].copy()

    # reset time
    df["t"] = df["t"] - t0

    df = df[df[posterior_cols].notna().any(axis=1)].copy()

    df.reset_index(drop=True, inplace=True)

    return df

def load_experiments(base_dir):
    """
    experiments[n_robots][likelihood][exp_dir] = {
        'params': dict,
        'runs': [df, df, ...]
    }
    """

    experiments = defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))

    run_files = glob.glob(os.path.join(base_dir, "**", "run_*.csv.gz"), recursive=True)

    for run_file in run_files:
        run_dir = os.path.dirname(run_file)

        params_file = os.path.join(run_dir, "params.txt")
        params = parse_params_file(params_file)

        n_robots = params.get("n_robots", "unknown")
        likelihood = params.get("likelihood_function", "unknown")

        key = run_dir

        if "runs" not in experiments[n_robots][likelihood][key]:
            experiments[n_robots][likelihood][key] = {
                "params": params,
                "runs": []
            }

        df = pd.read_csv(run_file, compression="gzip")

        df = cut_and_reindex_time(df)

        experiments[n_robots][likelihood][key]["runs"].append(df)

    return experiments


def concat_runs(run_list):
    if not run_list:
        return None
    return pd.concat(run_list, ignore_index=True)


# =========================================================
# LOAD DATA
# =========================================================

if not os.path.exists(base_dir):
    st.error("Directory does not exist")
    st.stop()

with st.spinner("Loading experiments..."):
    data = load_experiments(base_dir)

if not data:
    st.error("No experiments found")
    st.stop()


# =========================================================
# ALL EXPERIMENTS PLOT
# =========================================================

st.subheader("Correctness Boxplots over all settings")

stats = build_global_correctness_table(data)

for n in sorted(stats["n_robots"].unique()):

    sub = stats[stats["n_robots"] == n].copy()

    allowed_params = set()

    for lf, keys in LIKELIHOOD_SCHEMA.items():
        allowed_params.update(keys)

    # drop rows where param_id contains anything unexpected
    def is_valid_param(param_str):
        return any(k in param_str for k in allowed_params)

    sub = sub[sub["param_id"].apply(is_valid_param)]

    st.markdown(f"## n_robots = {n}")

    fig, ax = plt.subplots(figsize=(16, 6))

    sns.boxplot(
        data=sub,
        x="likelihood_function",
        y="correctness",
        hue="param_id",
        ax=ax
    )

    ax.set_ylim(0, 1)
    ax.tick_params(axis='x', rotation=25)

    ax.legend(
        bbox_to_anchor=(1.02, 1),
        loc="upper left",
        title="parameters"
    )

    st.pyplot(fig)

# =========================================================
# UI FILTERS
# =========================================================

st.subheader("Settings for single Experiment config")

n_robot_keys = sorted(data.keys())
selected_n = st.selectbox("n_robots", n_robot_keys)

likelihood_keys = sorted(data[selected_n].keys())
selected_lf = st.selectbox("likelihood_function", likelihood_keys)

experiments = data[selected_n][selected_lf]

exp_keys = list(experiments.keys())
selected_exp = st.selectbox("experiment group", exp_keys)

exp = experiments[selected_exp]

st.subheader("Parameters")
st.json(exp["params"])


# =========================================================
# LOAD RUN DATA
# =========================================================

runs = exp["runs"]

st.write(f"Runs found: {len(runs)}")

df_all = concat_runs(runs)

if df_all is None:
    st.warning("No data")
    st.stop()


# =========================================================
# BASIC ANALYSIS
# =========================================================

st.subheader("Data preview")
st.dataframe(df_all.head(200))


# =========================================================
# SINGLE EXPERIMENT PLOTS
# =========================================================

#if "robot" in df_all.columns:
#    st.subheader("Robot distribution")
#    st.bar_chart(df_all["robot"].value_counts())

stats_df = compute_correct_id_stats(runs)

st.subheader("Correct ID Statistics per Run / Robot")

if stats_df.empty:
    st.warning("No correct_ids data found.")
else:
    #st.dataframe(stats_df)

    pivot = stats_df.pivot(index="run", columns="robot", values="correctness")

    #st.subheader("Correctness per run / robot")
    st.bar_chart(pivot)

# =========================================================
# OPTIONAL: per-run inspection
# =========================================================

st.subheader("Single run inspection")

run_idx = st.number_input(
    "Run index",
    min_value=0,
    max_value=len(runs)-1,
    value=0
)

st.dataframe(runs[run_idx])
