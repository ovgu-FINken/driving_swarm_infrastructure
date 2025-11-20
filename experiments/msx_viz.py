import pandas as pd
import streamlit as st
from plotnine import *
import numpy as np

st.write("# analyze data from finding waldo")
data_file = st.file_uploader("upload file", type={"csv", "csv.gz"})

if data_file is not None:
    df = pd.read_csv(data_file)

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

    myplot = (

        ggplot(df, aes(
            x="t", y="sunburstError", color="robot", shape="robot"))
        + geom_point()
        + theme_light(base_size=11)
        + theme(legend_position='top',
                #axis_text_x=element_text(rotation=90,
                #                         hjust=0.5),
                figure_size=(6, 4.5),
                )
    )

    st.pyplot(ggplot.draw(myplot))

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
    myplot.save(f"figures/msx_line_error_dist.pdf", dpi=300, bbox_inches="tight")

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
    myplot.save(f"figures/msx_line_error_r.pdf", dpi=300, bbox_inches="tight")

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
            x="$d$", y="$|w - \\hat w|$", color="correct id", shape="robot"))
        + geom_point()
        + theme_light(base_size=11)
        + theme(legend_position='top',
                #axis_text_x=element_text(rotation=90,
                #                         hjust=0.5),
                figure_size=(6, 4.5),
                )
    )

    st.pyplot(ggplot.draw(myplot))


    myplot = (

        ggplot(df, aes(
            x="$\\theta$", y="$|w - \\hat w|$", color="correct id", shape="robot"))
        + geom_point()
        + theme_light(base_size=11)
        + theme(legend_position='top',
                #axis_text_x=element_text(rotation=90,
                #                         hjust=0.5),
                figure_size=(6, 4.5),
                )
    )

    st.pyplot(ggplot.draw(myplot))


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
