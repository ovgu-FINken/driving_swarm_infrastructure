#!/usr/bin/python3

import numbers
import yaml
import pandas as pd
from glob import glob
import os
import uuid
from pathlib import Path

def read_directory(directory: str):
    params = Path(directory) / "params.yaml"
    if not params.exists():
        print(f"skipping {directory}, because params.yaml does not exist")
        return None
    # read *csv.gz and params.yaml
    csv_files = glob(str(directory) + "/*.csv.gz")
    if not len(csv_files):
        return None
    with open(str(directory) + "/params.yaml", 'r') as f:
        params = yaml.load(f, Loader=yaml.SafeLoader)
    print(f"reading csv {csv_files[0]}")
    try:
        df = pd.read_csv(csv_files[0])
    except pd.errors.EmptyDataError:
        print(f"WARNING: Empty Data encountered in {csv_files[0]}")
        return pd.DataFrame()
    for k, v in params.items():
        if isinstance(v, numbers.Number):
            df[k] = v
        if isinstance(v, dict):
            s = []
            for kk, vv in v.items():
                df[f'{k}.{kk}'] = vv
                s.append(f"{kk}={vv}")
            df[k] = ",".join(s)
        else:
            df[k] = str(v)
    df["run_uuid"] = str(uuid.uuid4())
    df["x"] = df["x"].astype("float32") 
    df["y"] = df["y"].astype("float32") 
    df["cmd_vel_rot"] = df["cmd_vel_rot"].astype("float32") 
    df["cmd_vel_x"] = df["cmd_vel_x"].astype("float32")
    return df

def read_all_subdirectories(directory: str):
    if not Path(directory).exists():
        print(f"ERROR: path {directory} does not exist")
    dfs = []
    for subdirectory in glob(str(directory) + "*/**", recursive=True):
        if not os.path.isdir(subdirectory):
            continue
        #print(f"reading dir: {subdirectory}")
        df = read_directory(subdirectory)
        if df is not None:
            dfs.append(df)
    
    df = pd.concat(dfs, ignore_index=True)
    df.n = df.n.astype(int)
    df.run_uuid = df.run_uuid.astype(str)
    return df
