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

@st.cache_data
def load_data(dir=None):
    if not dir:
        #dir = "/home/semai/data/markov/params/experiment_2025-11-04/"
        dir = "/home/semai/data/markov/params/experiment_2025-11-08/"
    st.write(dir)
    df = read_all_subdirectories(dir)
    df["algorithm_variant"] = df.algorithm + ":" + df.algorithm_params
    df.n = df.n.astype("int")
    df = df.loc[~df["algorithm_params.beta"].eq(0.1)]
    return df

def aggregation(data):
    ret = data.groupby("nav/goal_completed").size().reset_index()[:-1].reindex()
    ret["ttg"] = ret[0]
    del ret[0]
    #st.dataframe(ret.head())
    return ret

def create_df_time(data):
    return data.groupby(["run_uuid","mode", "algorithm", "algorithm_variant", "n","robot"] + [param for param in data.columns if param.startswith("algorithm_params")], observed=True, dropna=False).apply(aggregation).reset_index()

df = load_data()
df = df.loc[df.command.eq("go")]
st.dataframe(df.head())
st.write(f"got data from {df.run_uuid.nunique()} runs with {df.algorithm_variant.nunique()} configurations of these algorithms: {df.algorithm.unique()}")
st.write(f"N={df.n.unique()}")



# def convert_coordinates(poly, resolution: float, oX: float, oY: float, width: float=0):
#     poly = poly * resolution + np.array([oX, oY])
#     poly[:, 1] *= -1
#     return Polygon(poly)
# 
# def read_map(info_file):
#     with open(info_file, 'r') as stream:
#         info = yaml.safe_load(stream)
#     img_file = info['image']
#     if img_file[0] not in ['/', '~']:
#         img_file = os.path.join(os.path.dirname(info_file), img_file)
#     img = io.imread(img_file).transpose()
#     img = np.array(img) # makes sure the data can be overwritten
#     return img, info
# 
# def read_obstacles(file_name):
#     img, info = read_map(file_name)
#     thresh = 100
#     contours = measure.find_contours(img, level=thresh)
#     unclassified = contours
#     obstacles, free = [], []
# 
#     # classify different levels of objects
#     # holes need to be filled by opposite value
#     oy = info['origin'][1] + info['resolution'] * img.shape[1]
#     while unclassified:
#         for i, poly in enumerate(unclassified):
#             if np.max(img[measure.grid_points_in_poly(img.shape, poly)]) <= thresh:
#                 img[measure.grid_points_in_poly(img.shape, poly)] = 200
#                 del unclassified[i]
#                 p = convert_coordinates(poly, info['resolution'], info['origin'][0], -1*oy)
#                 for f in free:
#                     if p.contains(f):
#                         p = p.difference(f)
#                 obstacles.append(p)
#                 break
# 
#             if np.min(img[measure.grid_points_in_poly(img.shape, poly)]) >= thresh:
#                 img[measure.grid_points_in_poly(img.shape, poly)] = 30
#                 del unclassified[i]
#                 p = convert_coordinates(poly, info['resolution'], info['origin'][0], -1*oy)
#                 for o in obstacles:
#                     if p.contains(o):
#                         p = p.difference(o)
#                 free.append(p)
#                 break
# 
#     return shapely.ops.unary_union(free), shapely.ops.unary_union(obstacles)
# 
# # Function to parse WKT and extract polygons
# def parse_multipolygon(wkt_str):
#     geom = wkt.loads(wkt_str)
#     polygons = []
#     if geom.geom_type == 'MultiPolygon':
#         for poly in geom.geoms:
#             coords = list(poly.exterior.coords)
#             polygons.append(coords)
#     elif geom.geom_type == 'Polygon':
#         coords = list(geom.exterior.coords)
#         polygons.append(coords)
#     return polygons


st.write("creating goal aggregate data")
df_ttg = create_df_time(df)
st.write("data for time to goal (ttg)")
st.dataframe(df_ttg.head())

st.write("creating run aggregated data")
df_per_robot = df.groupby(["run_uuid", "mode", "algorithm", "algorithm_variant", "n","robot"] + [param for param in df.columns if param.startswith("algorithm_params")], observed=True, dropna=False)["nav/goal_completed"].max().reset_index()
st.write("df_per_robot:")
st.dataframe(df_per_robot.head())
df_runs = df_per_robot.groupby(["run_uuid", "mode", "algorithm", "algorithm_variant", "n"] + [param for param in df_per_robot.columns if param.startswith("algorithm_params")], observed=True, dropna=False)["nav/goal_completed"].sum().reset_index()
df_runs.n = df_runs.n.astype('int')
#df_runs["algorithm_params.w_v"] = df_runs["algorithm_params.w_v"].astype('float') 
#df_ttg["algorithm_params.w_v"] = df_ttg["algorithm_params.w_v"].astype('float') 
st.write("df_runs")
st.dataframe(df_runs.head())

del df

#for n in range(df_ttg.n.max()):
#    st.write(f"ttg for {n} robots")
#    fig = px.box(df_ttg.loc[df_ttg.n.eq(n+1)], y="ttg", x="algorithm", color="algorithm", facet_row="mode")
#    #fig.update_yaxes(matches=None)
#    st.plotly_chart(fig)
#
#for n in range(df_runs.n.max()):
#    st.write(f"goals for {n} robots")
#    fig = px.box(df_runs.loc[df_runs.n.eq(n+1)], y="nav/goal_completed", x="algorithm", color="algorithm", facet_row="mode")
#    st.plotly_chart(fig)

#for n in range(df_runs.n.max()):
#    st.write(f"goal completed sim, n={n}")
#    fig = px.box(df_runs.loc[df_runs["mode"].eq("sim") & df_runs.n.eq(n+1)], y="nav/goal_completed", x="algorithm_params.w_r", color="algorithm_params.w_o", facet_col="algorithm_params.tau", facet_row="algorithm_params.w_v")
#    st.plotly_chart(fig)

#st.write(f"goal completed sim per robot")
#fig = px.box(df_runs.loc[df_runs["mode"].eq("sim")], y="nav/goal_completed", x="algorithm_params.r_goal", color="robot", facet_row="n")
#st.plotly_chart(fig)

#st.write(f"goal completed sim")
#fig = px.box(df_runs.loc[df_runs["mode"].eq("sim")], y="nav/goal_completed", x="algorithm_params.w_r", facet_row="n")
#st.plotly_chart(fig)

#st.write(f"goal completed fake")
#fig = px.box(df_runs.loc[df_runs["mode"].eq("sim")], y="nav/goal_completed", x="algorithm_params.beta", facet_row="n")
#st.plotly_chart(fig)

#for n in range(1,df_runs.n.max()):
#    fig = px.parallel_coordinates(df_runs.loc[df_runs.n.eq(n)], color="ttg", dimensions=[c for c in df_ttg.columns if c.startswith("algorithm_params")],
#                                 color_continuous_scale=px.colors.diverging.Tealrose,
#                                 color_continuous_midpoint=2)
#    st.plotly_chart(fig)
#map=read_obstacles("/home/semai/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/maps/icra2024.yaml")
#
#uff=to_wkt(map)
#
#st.write("Plotting:")
#fig = px.line(df, x="x", y="y", color="robot" , facet_row="run_uuid")
#
#for idx, wkt_str in enumerate(uff):
#    polygons = parse_multipolygon(wkt_str)
#    for polygon in polygons:
#        x, y = zip(*polygon)
#        # Add a filled polygon trace
#        fig.add_trace(go.Scatter(
#            x=x,
#            y=y,
#            fill='toself',
#            name=f'Polygon {idx+1}',
#            mode='lines',
#            line=dict(color='blue'),
#            fillcolor='rgba(0,0,255,0.3)',
#            showlegend=False
#        ), row='all', col='all', exclude_empty_subplots=True)
#


#
#fig.update_layout(height=5000)
#
#st.plotly_chart(fig)
#
#for col in df.columns:
#    st.write(f"{col}, type: {df[col].dtype}")

df_vis = df_runs.loc[df_runs["mode"].eq("sim")]

df_vis["algorithm_params.beta"] = df_vis["algorithm_params.beta"].fillna("n/a")
df_vis["algorithm"] = df_vis["algorithm"].map({"global_planner_baseline":"baseline", "global_planner_joint_markov":"$\\beta$-cooperation"})

df_vis["$\\beta$"] = df_vis["algorithm_params.beta"]
df_vis["$\\beta$"] = df_vis["$\\beta$"].astype('category')
st.write(df_vis.algorithm.unique())
myplot = (
        
ggplot(df_vis, aes(x="$\\beta$", y="nav/goal_completed", color="algorithm"))
    + geom_boxplot()
    # + scale_color_brewer(type='qual', palette='Dark2')
    + scale_color_manual(values=["#000000", "#999999", "#7570b3", "#e7298a"])
    + facet_grid("~n", labeller=lambda d: f"N={d}")
    + labs(y="goals completed", color="")
    + theme_light(base_size=11)
    + theme(legend_position='top',
            axis_text_x=element_text(rotation=90,
                                     hjust=0.5),
            figure_size=(6, 3),
           )
)

myplot.save("box_goals.pdf", dpi=300, bbox_inches="tight")
st.pyplot(ggplot.draw(myplot))
