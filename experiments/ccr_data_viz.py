from loader import read_all_subdirectories
import pandas as pd
import streamlit as st
import plotly.express as px


st.title('CCR Data Analyzer')
st.write('Load and visualize run data from driving swarm')

@st.cache_data
def load_data(dir=None):
    if not dir:
        dir = "/home/semai/data/2025-05-19/"
    st.write(dir)
    df = read_all_subdirectories(dir)
    df["algorithm_variant"] = df.algorithm + ":" + df.algorithm_params
    return df

df = load_data()
st.dataframe(df)
df = df.loc[df.command.eq("go")]

st.write("creating aggregate data")

def aggregation(data):
    ret = data.groupby("nav/goal_completed").size().reset_index()[:-1].reindex()
    ret["ttg"] = ret[0]
    del ret[0]
    #st.dataframe(ret.head())
    return ret

def create_df_time(data):
    return data.groupby(["run_uuid","mode", "algorithm", "algorithm_variant", "n","robot"], observed=True).apply(aggregation).reset_index()

df_ttg = create_df_time(df)
st.write("data for time to goal (ttg)")
st.dataframe(df_ttg)

fig = px.box(df_ttg, y="ttg", x="algorithm", color="algorithm", facet_row="mode", facet_col="n")
#fig.update_yaxes(matches=None)
st.plotly_chart(fig)

fig = px.box(df, y="nav/goal_completed", x="algorithm", color="algorithm", facet_row="mode", facet_col="n")
st.plotly_chart(fig)

for col in df.columns:
    st.write(f"{col}, type: {df[col].dtype}")