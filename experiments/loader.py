#!/usr/bin/python3

import numbers
import yaml
import pandas as pd
from glob import glob
import os
import uuid

def read_directory(directory: str):
    # read *csv.gz and params.yaml
    csv_files = glob(str(directory) + "/*.csv.gz")
    with open(str(directory) + "/params.yaml", 'r') as f:
        params = yaml.load(f, Loader=yaml.SafeLoader)
    df = pd.read_csv(csv_files[0])
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
    return df

def read_all_subdirectories(directory: str):
    dfs = []
    for subdirectory in glob(str(directory) + "/*"):
        if not os.path.isdir(subdirectory):
            continue
        dfs.append(read_directory(subdirectory))
    
    df = pd.concat(dfs, ignore_index=True)
    df.n = df.n.astype(int)
    df.run_uuid = df.run_uuid.astype(str)
    return df