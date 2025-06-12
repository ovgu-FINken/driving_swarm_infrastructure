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

def convert_coordinates(poly, resolution: float, oX: float, oY: float, width: float=0):
    poly = poly * resolution + np.array([oX, oY])
    poly[:, 1] *= -1
    return Polygon(poly)

def read_map(info_file):
    with open(info_file, 'r') as stream:
        info = yaml.safe_load(stream)
    img_file = info['image']
    if img_file[0] not in ['/', '~']:
        img_file = os.path.join(os.path.dirname(info_file), img_file)
    img = io.imread(img_file).transpose()
    img = np.array(img) # makes sure the data can be overwritten
    return img, info

def read_obstacles(file_name):
    img, info = read_map(file_name)
    thresh = 100
    contours = measure.find_contours(img, level=thresh)
    unclassified = contours
    obstacles, free = [], []

    # classify different levels of objects
    # holes need to be filled by opposite value
    oy = info['origin'][1] + info['resolution'] * img.shape[1]
    while unclassified:
        for i, poly in enumerate(unclassified):
            if np.max(img[measure.grid_points_in_poly(img.shape, poly)]) <= thresh:
                img[measure.grid_points_in_poly(img.shape, poly)] = 200
                del unclassified[i]
                p = convert_coordinates(poly, info['resolution'], info['origin'][0], -1*oy)
                for f in free:
                    if p.contains(f):
                        p = p.difference(f)
                obstacles.append(p)
                break

            if np.min(img[measure.grid_points_in_poly(img.shape, poly)]) >= thresh:
                img[measure.grid_points_in_poly(img.shape, poly)] = 30
                del unclassified[i]
                p = convert_coordinates(poly, info['resolution'], info['origin'][0], -1*oy)
                for o in obstacles:
                    if p.contains(o):
                        p = p.difference(o)
                free.append(p)
                break

    return shapely.ops.unary_union(free), shapely.ops.unary_union(obstacles)

# Function to parse WKT and extract polygons
def parse_multipolygon(wkt_str):
    geom = wkt.loads(wkt_str)
    polygons = []
    if geom.geom_type == 'MultiPolygon':
        for poly in geom.geoms:
            coords = list(poly.exterior.coords)
            polygons.append(coords)
    elif geom.geom_type == 'Polygon':
        coords = list(geom.exterior.coords)
        polygons.append(coords)
    return polygons


df_ttg = create_df_time(df)
st.write("data for time to goal (ttg)")
st.dataframe(df_ttg)

fig = px.box(df_ttg, y="ttg", x="algorithm", color="algorithm", facet_row="mode", facet_col="n")
#fig.update_yaxes(matches=None)
st.plotly_chart(fig)

fig = px.box(df, y="nav/goal_completed", x="algorithm", color="algorithm", facet_row="mode", facet_col="n")
st.plotly_chart(fig)

map=read_obstacles("/home/michael/driving_ws/src/driving_swarm_infrastructure/src/driving_swarm_bringup/maps/icra2024.yaml")

uff=to_wkt(map)

st.write("Plotting:")
fig = px.line(df, x="x", y="y", color="robot" , facet_row="run_uuid")

for idx, wkt_str in enumerate(uff):
    polygons = parse_multipolygon(wkt_str)
    for polygon in polygons:
        x, y = zip(*polygon)
        # Add a filled polygon trace
        fig.add_trace(go.Scatter(
            x=x,
            y=y,
            fill='toself',
            name=f'Polygon {idx+1}',
            mode='lines',
            line=dict(color='blue'),
            fillcolor='rgba(0,0,255,0.3)',
            showlegend=False
        ), row='all', col='all', exclude_empty_subplots=True)


fig.update_layout(height=5000)

st.plotly_chart(fig)

for col in df.columns:
    st.write(f"{col}, type: {df[col].dtype}")