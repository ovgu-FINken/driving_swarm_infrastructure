#!/usr/bin/python3

import yaml
import pandas as pd
from glob import glob
import os

def read_directory(directory: str):
    # read *csv.gz and params.yaml
    csv_files = glob(str(directory) + "/*.csv.gz")
    with open(str(directory) + "/params.yaml", 'r') as f:
        params = yaml.load(f, Loader=yaml.SafeLoader)
    df = pd.read_csv(csv_files[0])
    for key, value in params.items():
        # if we have a nested dict, we use dot notation
        if type(value) == dict:
            for k, v in value.items():
                df[f"{key}.{k}"] = v
        else:
            df[key] = value
    return df

def read_all_subdirectories(directory: str):
    dfs = []
    for subdirectory in glob(str(directory) + "/*"):
        if not os.path.isdir(subdirectory):
            continue
        dfs.append(read_directory(subdirectory))
    return pd.concat(dfs, ignore_index=True)