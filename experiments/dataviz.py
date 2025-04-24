from scipy import stats
import streamlit as st
import pandas as pd
import numpy as np
import plotly.express as px
import matplotlib.pyplot as plt

st.title('Data Visualization')

df = pd.read_csv("example_data.csv.gz")
df = df.loc[df.command.eq("go")]

st.write("Tabular data in Dataframe:")
st.dataframe(df)

st.write("Plotting:")
fig = px.line(df, x="x", y="y", color="robot")
st.plotly_chart(fig)

ttg = df.groupby(["robot", "nav/goal_completed"]).size().reset_index()[:-1].reindex()
ttg["ttg"] = ttg[0]
del ttg[0]
st.dataframe(ttg)

fig = px.box(ttg, x="robot", y="ttg")
st.plotly_chart(fig)

st.write("Speed:")
fig = px.histogram(df, color="robot", y="cmd_vel_x")
st.plotly_chart(fig)
