import os
import sys
import matplotlib
matplotlib.use("Agg")
import pandas as pd
from plotnine import *
import numpy as np

data_file = sys.argv[1] if len(sys.argv) > 1 else "./dars26/Hyperparameters/n_robots__5/dist_threshold__1_025/angle_threshold__0_08727/old_error_penalty_scale__1_05/run_2.csv.gz"
compression = "gzip" if data_file.endswith(".gz") else None
df = pd.read_csv(data_file, compression=compression)

os.makedirs("figures", exist_ok=True)

df["$x_w$"] = df.real_waldo_pos_0
df["$y_w$"] = df.real_waldo_pos_1
df["$\\hat x_w$"] = df["sunburstRobotCalc/waldoPosition_0"]
df["$\\hat y_w$"] = df["sunburstRobotCalc/waldoPosition_1"]
df["correct id"] = df["identify_as"] == df["robot"]

df["$|w - \\hat w|$"] = np.sqrt((df["$x_w$"] - df["$\\hat x_w$"])**2 + (df["$y_w$"] - df["$\\hat y_w$"])**2)
df["$d$"] = np.sqrt((df["$x_w$"] + df["$y_w$"])**2)
df["$\\hat d$"] = np.sqrt((df["$\\hat x_w$"] + df["$\\hat y_w$"])**2)
df["$d - \\hat d$"] = df["$d$"] - df["$\\hat d$"]
df["$\\theta$"] = np.arctan2(df["$y_w$"], df["$x_w$"])
df["$\\hat \\theta$"] = np.arctan2(df["$\\hat y_w$"], df["$\\hat x_w$"])
df["$\\theta - \\hat \\theta$"] = df["$\\theta$"] - df["$\\hat \\theta$"]
df["$\\theta - \\hat \\theta$"] = (
    (df["$\\theta - \\hat \\theta$"] + np.pi) % (2 * np.pi) - np.pi
)
df["ND"] = df["num_detections"].astype("category")

myplot = (
    ggplot(df, aes(x="$|w - \\hat w|$", fill="robot"))
    + geom_histogram(binwidth=0.1)
    + theme_light(base_size=11)
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_error_eucleadian_histogram.png", dpi=300, bbox_inches="tight")
print("Saved msx_error_eucleadian_histogram.png")

myplot = (
    ggplot(df, aes(x="sunburstError", fill="robot"))
    + geom_histogram(binwidth=0.1)
    + theme_light(base_size=11)
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_error_sunburst_histogram.png", dpi=300, bbox_inches="tight")
print("Saved msx_error_sunburst_histogram.png")

myplot = (
    ggplot(df, aes(x="t", y="$|w - \\hat w|$", color="correct id", shape="robot"))
    + geom_point()
    + theme_light(base_size=11)
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_euclidean_error_time.png", dpi=300, bbox_inches="tight")
print("Saved msx_euclidean_error_time.png")

myplot = (
    ggplot(df, aes(x="t", y="sunburstError", color="robot", shape="robot"))
    + geom_point()
    + theme_light(base_size=11)
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_sunburst_error_time.png", dpi=300, bbox_inches="tight")
print("Saved msx_sunburst_error_time.png")

myplot = (
    ggplot(df, aes(x="t", y="$d - \\hat d$", color="robot", shape="robot"))
    + geom_line()
    + geom_point()
    + theme_light(base_size=11)
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_line_error_distance.png", dpi=300, bbox_inches="tight")
print("Saved msx_line_error_distance.png")

myplot = (
    ggplot(df, aes(x="t", y="$\\theta - \\hat \\theta$", color="robot", shape="robot"))
    + geom_line()
    + geom_point()
    + theme_light(base_size=11)
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_line_error_theta.png", dpi=300, bbox_inches="tight")
print("Saved msx_line_error_theta.png")

myplot = (
    ggplot(df, aes(x="ND", y="$|w - \\hat w|$", color="correct id"))
    + geom_boxplot()
    + theme_light(base_size=11)
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_boxplot_detection_count.png", dpi=300, bbox_inches="tight")
print("Saved msx_boxplot_detection_count.png")

posterior_cols = [f"posterior_{i}" for i in range(df["robot"].nunique())]
df_post = df.melt(
    id_vars=["t", "robot"],
    value_vars=posterior_cols,
    var_name="hypothesis",
    value_name="posterior"
)
df_post["hypothesis"] = df_post["hypothesis"].str.replace("posterior_", "").astype(int)

robots_ordered = sorted(df_post["robot"].unique())
n_hypotheses = df_post["hypothesis"].nunique()

robot_letter = lambda r: r.replace("robot", "").upper()
df_post["row_label"] = df_post["robot"].apply(robot_letter) + " / " + df_post["hypothesis"].astype(str)
row_order = [
    f"{robot_letter(r)} / {h}"
    for r in reversed(robots_ordered)
    for h in reversed(range(n_hypotheses))
]
df_post["row_label"] = pd.Categorical(df_post["row_label"], categories=row_order, ordered=True)

separator_positions = [n_hypotheses * i + 0.5 for i in range(1, len(robots_ordered))]
n_rows = len(robots_ordered) * n_hypotheses
fig_height = max(4, n_rows * 0.25)

myplot = (
    ggplot(df_post, aes(x="t", y="row_label", fill="posterior"))
    + geom_tile()
    + geom_hline(yintercept=separator_positions, color="black", size=1.5)
    + scale_fill_gradient(low="#F4B183", high="#5B9BD5", name="Posterior")
    + labs(x="Time Step", y="Robot ID / Assignment")
    + theme_light(base_size=10)
    + theme(
        figure_size=(14, fig_height * 0.75),
        axis_text_x=element_text(size=10, fontweight="bold"),
        axis_text_y=element_text(size=10, fontweight="bold"),
        axis_title_x=element_text(size=10, fontweight="bold"),
        axis_title_y=element_text(size=10, fontweight="bold"),
        legend_title=element_text(size=12, fontweight="bold"),
        legend_text=element_text(size=12, fontweight="bold"),
    )
)
myplot.save("figures/msx_posterior_heatmap.png", dpi=300, bbox_inches="tight")
print("Saved msx_posterior_heatmap.png")

myplot = (
    ggplot(df, aes(x="$\\theta$", y="$|w - \\hat w|$", color="correct id"))
    + geom_point()
    + theme_light(base_size=7)
    + facet_wrap("~robot")
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_line_error_euclidean_to_theta.png", dpi=300, bbox_inches="tight")
print("Saved msx_line_error_euclidean_to_theta.png")

myplot = (
    ggplot(df, aes(x="$d$", y="$|w - \\hat w|$", color="correct id"))
    + geom_point()
    + theme_light(base_size=7)
    + facet_wrap("~robot")
    + theme(legend_position="top", figure_size=(14, 4.5))
)
myplot.save("figures/msx_line_error_euclidian_to_distance.png", dpi=300, bbox_inches="tight")
print("Saved msx_line_error_euclidian_to_distance.png")