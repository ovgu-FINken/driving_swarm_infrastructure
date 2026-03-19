import pandas as pd
import streamlit as st
import numpy as np
import plotly.graph_objects as go
import time

st.set_page_config(layout="wide")
st.title("Analyze Data from Finding Waldo")

# File uploader
data_file = st.file_uploader("Upload CSV file", type=["csv", "csv.gz"], key="waldo_file")

if data_file is not None:
    df = pd.read_csv(data_file)

    # Posterior color
    robot_to_idx = {"robotA": 0, "robotB": 1, "robotC": 2, "robotD": 3}
    df["posterior_color"] = df.apply(
        lambda row: row[f"posterior_{robot_to_idx[row['robot']]}"], axis=1
    )

    # Fixed axes
    all_x = df["x"]
    all_y = df["y"]
    margin = 0.1 * max(all_x.max() - all_x.min(), all_y.max() - all_y.min())
    x_range = [all_x.min() - margin, all_x.max() + margin]
    y_range = [all_y.min() - margin, all_y.max() + margin]

    # Time range
    t_min, t_max = int(df["t"].min()), int(df["t"].max())

    # Sidebar controls
    st.sidebar.header("Controls")
    time_slider = st.sidebar.slider("Time", min_value=t_min, max_value=t_max, value=t_min, step=1, key="time_slider")
    play_button = st.sidebar.button("Play / Pause", key="play_button")

    if "playing" not in st.session_state:
        st.session_state.playing = False

    if play_button:
        st.session_state.playing = not st.session_state.playing

    robots = df["robot"].unique()
    plot_placeholder = st.empty()  # Plot-Container

    # --- Initial Figure ---
    fig = go.Figure(layout=dict(
        xaxis=dict(title="x", scaleanchor="y", zeroline=True, range=x_range),
        yaxis=dict(title="y", scaleanchor="x", zeroline=True, range=y_range),
        width=700,
        height=700,
        plot_bgcolor="white",
        paper_bgcolor="white",
        title=f"Robot positions at t={time_slider}"
    ))

    # Trajektorien nur einmal zeichnen
    for r in robots:
        df_r = df[df["robot"] == r]
        fig.add_trace(go.Scatter(
            x=df_r["x"],
            y=df_r["y"],
            mode="lines",
            line=dict(color='black', width=1, dash='dot'),
            showlegend=False
        ))

    # Marker für aktuelle Position initialisieren
    marker_traces = []
    for r in robots:
        df_r_current = df[df["t"] == time_slider][df["robot"] == r]
        trace = go.Scatter(
            x=df_r_current["x"],
            y=df_r_current["y"],
            mode="markers",
            marker=dict(
                size=12,
                color=df_r_current["posterior_color"],
                colorscale="Viridis",
                cmin=0, cmax=1,
                colorbar=dict(title="Posterior")
            ),
            name=f"{r} pos"
        )
        fig.add_trace(trace)
        marker_traces.append(trace)

    # Initial plot
    plot_placeholder.plotly_chart(fig, use_container_width=True)

    # --- Play-Funktion ---
    if st.session_state.playing:
        for t in range(time_slider + 1, t_max + 1):
            # Update marker traces nur
            for i, r in enumerate(robots):
                df_r_current = df[df["t"] == t][df["robot"] == r]
                if not df_r_current.empty:
                    marker_traces[i].x = df_r_current["x"]
                    marker_traces[i].y = df_r_current["y"]
                    marker_traces[i].marker.color = df_r_current["posterior_color"]

            # Plot updaten
            plot_placeholder.plotly_chart(fig, use_container_width=True)
            time.sleep(0.1)
            if not st.session_state.playing:
                break
