import pandas as pd
import streamlit as st
from plotnine import *
import numpy as np

st.write("# analyze data from finding waldo")
data_file = st.file_uploader("upload file", type={"csv", "csv.gz"})
if data_file is not None:
    df = pd.read_csv(data_file)
else:
    df = pd.DataFrame()
    

st.write("got the data here:")
st.dataframe(df)

df["real $x_w$"] = df.real_waldo_pos_0
df["real $y_w$"] = df.real_waldo_pos_1
df["estimate $x_w$"] = df["sunburstRobotCalc/waldoPosition_0"]
df["estimate $y_w$"] = df["sunburstRobotCalc/waldoPosition_1"]

df["dist"] = np.sqrt((df["real $x_w$"]-df["estimate $x_w$"])**2 + (df["real $y_w$"] - df["estimate $x_w$"])**2)
df["real $r$"] = np.sqrt((df["real $x_w$"] + df["real $y_w$"])**2)
df["estimate $r$"] = np.sqrt((df["estimate $x_w$"] + df["estimate $y_w$"])**2)
df["error $r$"] = df["real $r$"] - df["estimate $r$"]
df["real $\\theta$"] = np.arctan2(df["real $y_w$"], df["real $x_w$"])
df["estimate $\\theta$"] = np.arctan2(df["estimate $y_w$"], df["estimate $x_w$"])
df["error $\\theta$"] = df["real $\\theta$"] - df["estimate $\\theta$"]

# start plotting stuff
myplot = (

    ggplot(df, aes(
        x="sunburstError", fill="robot"))
    + geom_histogram()
    + theme_light(base_size=11)
    + theme(legend_position='top',
            axis_text_x=element_text(rotation=90,
                                     hjust=0.5),
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
        x="t", y="dist", color="robot", shape="robot"))
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
        x="t", y="error $r$", color="robot", shape="robot"))
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
        x="t", y="error $\\theta$", color="robot", shape="robot"))
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


#myplot.save(f"msx_histogram.pdf", dpi=300, bbox_inches="tight")