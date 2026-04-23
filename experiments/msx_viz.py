import os.path

import pandas as pd
import streamlit as st
from plotnine import *
import numpy as np

st.write("# analyze data from finding waldo")
data_file = st.file_uploader("upload file", type={"csv", "csv.gz"})

if data_file is not None:
    compression = 'gzip' if data_file.name.endswith('.gz') else None
    df = pd.read_csv(data_file, compression=compression)
elif os.path.exists('../data.csv.gz'):
    df = pd.read_csv('../data.csv.gz', compression='gzip')
else:
    st.stop()

# df = df.dropna(subset=["real_waldo_pos_0", "real_waldo_pos_1"])

df["$x_w$"] = df.real_waldo_pos_0
df["$y_w$"] = df.real_waldo_pos_1
df["$\\hat x_w$"] = df["sunburstRobotCalc/waldoPosition_0"]
df["$\\hat y_w$"] = df["sunburstRobotCalc/waldoPosition_1"]
df["correct id"] = df["identify_as"] == df["robot"]

df["$|w - \\hat w|$"] = np.sqrt((df["$x_w$"]-df["$\\hat x_w$"])**2 + (df["$y_w$"] - df["$\\hat y_w$"])**2)
df["$d$"] = np.sqrt((df["$x_w$"] + df["$y_w$"])**2)
df["$\\hat d$"] = np.sqrt((df["$\\hat x_w$"] + df["$\\hat y_w$"])**2)
df["$d - \\hat d$"] = df["$d$"] - df["$\\hat d$"]
df["$\\theta$"] = np.arctan2(df["$y_w$"], df["$x_w$"])
df["$\\hat \\theta$"] = np.arctan2(df["$\\hat y_w$"], df["$\\hat x_w$"])
df["$\\theta - \\hat \\theta$"] = df["$\\theta$"] - df["$\\hat \\theta$"]
# Wrap auf (-pi, pi]
df["$\\theta - \\hat \\theta$"] = (
    (df["$\\theta - \\hat \\theta$"] + np.pi) % (2 * np.pi) - np.pi
)
df["ND"] = df["num_detections"].astype("category")

st.write("got the data here:")
st.dataframe(df)

# start plotting stuff
myplot = (

    ggplot(df, aes(
        x="$|w - \\hat w|$", fill="robot"))
    + geom_histogram(binwidth=0.1)
    + theme_light(base_size=11)
    + theme(legend_position='top',
            figure_size=(6, 4.5),
            )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_error_eucleadian_histogram.pdf", dpi=300, bbox_inches="tight")

myplot = (

        ggplot(df, aes(
            x="sunburstError", fill="robot"))
        + geom_histogram(binwidth=0.1)
        + theme_light(base_size=11)
        + theme(legend_position='top',
                figure_size=(6, 4.5),
                )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_error_sunburst_histogram.pdf", dpi=300, bbox_inches="tight")

myplot = (

    ggplot(df, aes(
        x="t", y="$|w - \\hat w|$", color="correct id", shape="robot"))
    + geom_point()
    + theme_light(base_size=11)
    + theme(legend_position='top',
            #axis_text_x=element_text(rotation=90,
            #                         hjust=0.5),
            figure_size=(6, 4.5),
            )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_euclidean_error_time.pdf", dpi=300, bbox_inches="tight")

myplot = (

        ggplot(df, aes(
            x="t", y="sunburstError", color="robot", shape="robot"))
        + geom_point()
        + theme_light(base_size=11)
        + theme(legend_position='top',
                # axis_text_x=element_text(rotation=90,
                #                         hjust=0.5),
                figure_size=(6, 4.5),
                )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_sunburst_error_time.pdf", dpi=300, bbox_inches="tight")

myplot = (

    ggplot(df, aes(
        x="t", y="$d - \\hat d$", color="robot", shape="robot"))
    + geom_line()
    + geom_point()
    + theme_light(base_size=11)
    + theme(legend_position='top',
            #axis_text_x=element_text(rotation=90,
            #                         hjust=0.5),
            figure_size=(6, 4.5),
            )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_line_error_distance.pdf", dpi=300, bbox_inches="tight")


myplot = (

    ggplot(df, aes(
        x="t", y="$\\theta - \\hat \\theta$", color="robot", shape="robot"))
    + geom_line()
    + geom_point()
    + theme_light(base_size=11)
    + theme(legend_position='top',
            #axis_text_x=element_text(rotation=90,
            #                         hjust=0.5),
            figure_size=(6, 4.5),
            )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_line_error_theta.pdf", dpi=300, bbox_inches="tight")


myplot = (

    ggplot(df, aes(
        x="ND", y="$|w - \\hat w|$", color="correct id"))
    + geom_boxplot()
    + theme_light(base_size=11)
    + theme(legend_position='top',
            #axis_text_x=element_text(rotation=90,
            #                         hjust=0.5),
            figure_size=(6, 4.5),
            )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_boxplot_detection_count.pdf", dpi=300, bbox_inches="tight")


# reshape posterior columns to long format
posterior_cols = [f"posterior_{i}" for i in range(df["robot"].nunique())]

df_post = df.melt(
    id_vars=["t", "robot"],
    value_vars=posterior_cols,
    var_name="hypothesis",
    value_name="posterior"
)

# extract hypothesis id
df_post["hypothesis"] = df_post["hypothesis"].str.replace("posterior_", "").astype(int)

#st.dataframe(df_post)

# # plot posterior over time, faceted by robot
# myplot = (
#     ggplot(df_post, aes(
#         x="t",
#         y="posterior",
#         color="factor(hypothesis)"
#     ))
#     + geom_line()
#     + facet_wrap("~robot")
#     + theme_light(base_size=8)
#     #+ scale_y_log10()
#     + theme(
#         legend_position='top',
#         figure_size=(6, 4.5),
#     )
# )
#
# st.pyplot(ggplot.draw(myplot))
# myplot.save(
#     "figures/msx_posterior_over_time.pdf",
#     dpi=300,
#     bbox_inches="tight"
# )



# # reshape likelihood columns to long format
# likelihood_cols = [f"likelihoods_{i}" for i in range(df["robot"].nunique())]
#
# df_like = df.melt(
#     id_vars=["t", "robot"],
#     value_vars=likelihood_cols,
#     var_name="hypothesis",
#     value_name="likelihood"
# )
# df_like["hypothesis"] = df_like["hypothesis"].str.replace("likelihoods_", "").astype(int)
#
# myplot = (
#     ggplot(df_like, aes(
#         x="t",
#         y="likelihood",
#         color="factor(hypothesis)"
#     ))
#     + geom_line()
#     + facet_wrap("~robot")
#     + theme_light(base_size=8)
#     + theme(
#         legend_position='top',
#         figure_size=(6, 4.5),
#     )
# )
#
# st.pyplot(ggplot.draw(myplot))
# myplot.save(
#     "figures/msx_likelihoods_over_time.pdf",
#     dpi=300,
#     bbox_inches="tight"
# )



# df_entropy = df_post.copy()
# df_entropy["entropy"] = -df_entropy["posterior"] * np.log(df_entropy["posterior"] + 1e-12)
#
# df_entropy = df_entropy.groupby(["t", "robot"])["entropy"].sum().reset_index()
#
# myplot = (
#     ggplot(df_entropy, aes(x="t", y="entropy", color="robot"))
#     + geom_line()
#     + theme_light(base_size=8)
#     + theme(
#         legend_position='top',
#         figure_size=(6, 4.5),
#     )
# )
#
# st.pyplot(ggplot.draw(myplot))
# myplot.save(
#     "figures/msx_entropy.pdf",
#     dpi=300,
#     bbox_inches="tight"
# )



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

# horizontal lines between robot groups
separator_positions = [n_hypotheses * i + 0.5 for i in range(1, len(robots_ordered))]

n_rows = len(robots_ordered) * n_hypotheses
fig_height = max(4, n_rows * 0.25)

myplot = (
    ggplot(df_post, aes(
        x="t",
        y="row_label",
        fill="posterior"
    ))
    + geom_tile()
    + geom_hline(
        yintercept=separator_positions,
        color="black",
        size=1.5
    )
    + scale_fill_gradient(low="white", high="steelblue")
    + labs(x="Frame", y="Robot ID / Assignment")
    + theme_light(base_size=8)
    + theme(
        figure_size=(8, fig_height),
        axis_text_y=element_text(size=6),
    )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(
    "figures/msx_posterior_heatmap.pdf",
    dpi=300,
    bbox_inches="tight"
)

myplot = (

    ggplot(df, aes(
        x="$\\theta$", y="$|w - \\hat w|$", color="correct id"))
    + geom_point()
    + theme_light(base_size=7)
    + facet_wrap("~robot")
    + theme(legend_position='top',
            #axis_text_x=element_text(rotation=90,
            #                         hjust=0.5),
            figure_size=(6, 4.5),
            )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_line_error_euclidean_to_theta.pdf", dpi=300, bbox_inches="tight")


myplot = (

    ggplot(df, aes(
        x="$d$", y="$|w - \\hat w|$", color="correct id"))
    + geom_point()
    + theme_light(base_size=7)
    + facet_wrap("~robot")
    + theme(legend_position='top',
            #axis_text_x=element_text(rotation=90,
            #                         hjust=0.5),
            figure_size=(6, 4.5),
            )
)

st.pyplot(ggplot.draw(myplot))
myplot.save(f"figures/msx_line_error_euclidian_to_distance.pdf", dpi=300, bbox_inches="tight")