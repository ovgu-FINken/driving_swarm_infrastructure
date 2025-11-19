from shapely.io import to_wkt
from shapely import Polygon, Point, MultiLineString, LineString, simplify, union, wkt
from loader import read_all_subdirectories
from skimage import io, measure
import os
import pandas as pd
import streamlit as st
import plotly.express as px
import yaml
import numpy as np
import shapely.geometry
import plotly.graph_objects as go
import shapely.ops
from plotnine import *


st.title('CCR Data Analyzer')
st.write('Load and visualize run data from driving swarm')

def load_data(dir):
    st.write(dir)
    ret = read_all_subdirectories(dir)
    ret["algorithm_variant"] = ret.algorithm + ":" + ret.algorithm_params
    
    return ret

def aggregation(data):
    ret = data.groupby("nav/goal_completed").size().reset_index()[:-1].reindex()
    ret["ttg"] = ret[0]
    del ret[0]
    #st.dataframe(ret.head())
    return ret

def create_df_time(data):
    return data.groupby(["run_uuid","mode", "algorithm", "algorithm_variant", "n","robot"] + [param for param in data.columns if param.startswith("algorithm_params")], observed=True, dropna=False).apply(aggregation).reset_index()

@st.cache_data
def create_goal_data(dir, scenario):
    #dir = "/home/semai/data/markov/params/experiment_2025-11-04/"
    df = load_data(dir)
    df["scenario"] = scenario
    st.write(f"len {scenario}: {len(df)}")
    df = df.loc[df.command.eq("go")]
    categorial_columns = ["algorithm", "robot", "algorithm_variant", "run_uuid", "mode"]# + [param for param in df.columns if param.startswith("algorithm_params")]
    for c in categorial_columns:
        df[c] = df[c].astype('category')
    df = df.loc[~df["algorithm_params.beta"].eq(0.1)]
    df.n = df.n.astype("int")

    st.dataframe(df.head())
    st.write(f"got data from {df.run_uuid.nunique()} runs with {df.algorithm_variant.nunique()} configurations of these algorithms: {df.algorithm.unique()}")
    st.write(f"N={df.n.unique()}")
    s_dict = { s: len(df.loc[df.scenario.eq(s)].run_uuid.unique()) for s in df.scenario.unique()}
    st.write(f"scnerios: {s_dict}")

    #st.write("creating goal aggregate data")
    #df_ttg = create_df_time(df)
    #st.write("data for time to goal (ttg)")
    #st.dataframe(df_ttg.head())

    st.write("creating run aggregated data")
    df_per_robot = df.groupby(["run_uuid", "mode", "algorithm", "algorithm_variant", "n","robot","scenario"] + [param for param in df.columns if param.startswith("algorithm_params")], observed=True, dropna=False)["nav/goal_completed"].max().reset_index()
    del df
    st.write("df_per_robot:")
    st.dataframe(df_per_robot.head())
    df_runs = df_per_robot.groupby(["run_uuid", "mode", "algorithm", "algorithm_variant", "n", "scenario"] + [param for param in df_per_robot.columns if param.startswith("algorithm_params")], observed=True, dropna=False)["nav/goal_completed"].sum().reset_index()
    df_runs.n = df_runs.n.astype('int')
    st.write("df_runs")
    st.dataframe(df_runs.head())

    return df_runs


dirs = {
    "/home/semai/data/markov/paper/experiment_2025-11-04/": "1m grid",
    "/home/semai/data/markov/paper/easy1/": "0.5m grid",
    "/home/semai/data/markov/paper/easy2/": "0.5m grid",
    "/home/semai/data/markov/paper/1m_10/": "1m grid",
}
df_runs = pd.concat([create_goal_data(dir, scenario) for dir, scenario in dirs.items()])
df_runs["scenario"] = df_runs["scenario"].astype('category')

df_vis = df_runs.loc[df_runs["mode"].eq("sim")]

df_vis["algorithm_params.beta"] = df_vis["algorithm_params.beta"].fillna("n/a")
df_vis["algorithm"] = df_vis["algorithm"].map({"global_planner_baseline":"baseline", "global_planner_joint_markov":"$\\beta$-cooperation"})

df_vis["$\\beta$"] = df_vis["algorithm_params.beta"]
df_vis["$\\beta$"] = df_vis["$\\beta$"].astype('category')

for scenario in ["1m grid", "0.5m grid"]:

    st.write(df_vis.algorithm.unique())
    myplot = (
            
    ggplot(df_vis.loc[df_vis.scenario.eq(scenario)], aes(x="$\\beta$", y="nav/goal_completed", color="algorithm"))
        + geom_boxplot()
        #+ geom_violin()
        # + scale_color_brewer(type='qual', palette='Dark2')
        + scale_color_manual(values=["#000000", "#999999", "#7570b3", "#e7298a"])
        + facet_wrap("n", ncol=5, labeller=lambda d: f"N={d}")
        + labs(y="goals completed", color="")
        + theme_light(base_size=11)
        + theme(legend_position='top',
                axis_text_x=element_text(rotation=90,
                                         hjust=0.5),
                figure_size=(6, 4.5),
               )
    )

    st.pyplot(ggplot.draw(myplot))
    myplot.save(f"box_goals_{scenario}.pdf", dpi=300, bbox_inches="tight")

from scipy.stats import iqr, mannwhitneyu
# tabular comparison
st.write("compute comparison frame")
compare = df_vis.groupby(["n", "scenario", "algorithm"])["nav/goal_completed"].agg(["mean", "std", "median", iqr, "count"]).reset_index()
compare["mean_std"] = compare.apply(
    lambda r: f"{r['mean']:.2f} ({r['std']:.2f})", axis=1
)
compare["median_iqr"] = compare.apply(
    lambda r: f"{r['median']:.2f} ({r['iqr']:.2f})", axis=1
)

st.dataframe(compare)
st.write("this is the end my friend")

## Chatty:

alg_a = "baseline"
alg_b = "$\\beta$-cooperation"


# Step 1: Compute summary statistics
compare = (
    df_vis.groupby(["n", "scenario", "algorithm"])["nav/goal_completed"]
    .agg(mean="mean", std="std", median="median", iqr=iqr, count="count")
    .reset_index()
)

# Step 2: Compute Mann–Whitney U p-values per (n, scenario)
def mannwhitney_row(g):
    x = g.loc[g["algorithm"] == alg_a, "nav/goal_completed"]
    y = g.loc[g["algorithm"] == alg_b, "nav/goal_completed"]
    if len(x) > 0 and len(y) > 0:
        u, p = mannwhitneyu(x, y, alternative="two-sided")
    else:
        p = np.nan
    return pd.Series({"p_value": p})

p_values = df_vis.groupby(["n", "scenario"]).apply(mannwhitney_row).reset_index()
compare = compare.merge(p_values, on=["n", "scenario"], how="left")

# Step 3: Build formatted rows
rows = []
for (n, scenario), g in compare.groupby(["n", "scenario"]):
    if len(g["algorithm"].unique()) < 2:
        continue

    a = g[g["algorithm"] == alg_a].iloc[0]
    b = g[g["algorithm"] == alg_b].iloc[0]
    p = a["p_value"]

    def fmt(entry):
        return f"{entry['mean']:.2f} $\\pm$ {entry['std']:.2f}"# | {entry['median']:.2f} ({entry['iqr']:.2f})"

    val_a = fmt(a)
    val_b = fmt(b)

    # Bold best mean if significant
    if not np.isnan(p) and p < 0.01:
        if a["mean"] > b["mean"]:
            val_a = f"\\textbf{{{val_a}}}"
        else:
            val_b = f"\\textbf{{{val_b}}}"

    rows.append({
        "n": n,
        "scenario": scenario,
        alg_a: val_a,
        alg_b: val_b,
        "p": f"{p:.2e}" if not np.isnan(p) else "--"
    })

table = pd.DataFrame(rows)

# Step 4: Pivot so “0.5m grid” and “1m grid” appear side-by-side
wide = (
    table.pivot(index="n", columns="scenario", values=[alg_a, alg_b, "p"])
    .swaplevel(axis=1)
    .sort_index(axis=1)
)
wide.columns = [f"{sc} {alg}" for sc, alg in wide.columns]

# Step 5: Output LaTeX
latex = wide.to_latex(escape=False, na_rep="--", column_format="lcccccc")

print(latex)
