import pandas as pd
import streamlit as st
from plotnine import *

st.write("# analyze data from finding waldo")
data_file = st.file_uploader("upload file", type={"csv", "csv.gz"})
if data_file is not None:
    df = pd.read_csv(data_file)
else:
    df = pd.DataFrame()

st.write("got the data here:")
st.dataframe(df)

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
#myplot.save(f"msx_histogram.pdf", dpi=300, bbox_inches="tight")

