from multiprocessing.pool import Pool
import numbers
import os
import argparse
import pandas.io.sql as pdsql
import yaml
import numpy as np
import pandas as pd
import tqdm
from data_export import read_rosbag_all_in_one,  DataConverter
from glob import glob
import logging
from functools import partial

def get_db3_files_in_folders(directory):
    return glob(f'{directory}/**/*.db3', recursive=True)

def aggregate_file(db3_file, use_cached=True):
    # we check if a pkl file already exists
    # this caches the result so we dont have to recompute this every time
    
    if use_cached and os.path.isfile(db3_file.replace('.db3', '.pkl')):
        logging.info(f'found pkl for {db3_file} not recomputing')
        return pd.read_pickle(db3_file.replace('.db3', '.pkl'))
    try:
        logging.info(f'start aggregating {db3_file}')
        topics = ["%goal_completed", "/tf", "/clock", "/command", "%status", "%cmd_vel", "%current_node"]
        data = read_rosbag_all_in_one(db3_file, topics=topics)
        logging.info(f'table aggregating {db3_file}')
        converter = DataConverter()
        df = converter.convert_df(data)
        df['db3'] = db3_file
        # read experiment ../params.yaml which is stored next to the rosbag folder
        # get path for params.yaml
        params_file = os.path.join(os.path.dirname(db3_file), '..', 'params.yaml')
        logging.info(f'reading params from {params_file}')
        with open(params_file) as f:
            params = yaml.safe_load(f)
        print(params)
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

        df.to_pickle(db3_file.replace('.db3', '.pkl'))
        logging.info(f'done  aggregating {db3_file}')
        return df
    except pdsql.DatabaseError:
        print(f'error aggregating {db3_file}')
        return pd.DataFrame()

def get_robot_id(x, y, start_pos):
    dist = [np.linalg.norm(np.array([x-p[0], y-p[1]])) for p in start_pos]
    return int(np.argmin(np.array(dist)))

# because we switched turtlebots during experiments we have to assign new names and pairs by the robots starting position
def data_assignments(dfs, db3_files, pos_file=None):
    if pos_file is None: 
        pos_file = '~/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/params/icra2024_waypoints.yaml'
    with open(pos_file, 'r') as file:
        waypoints = yaml.safe_load(file)
    starting_positions = [w['waypoints'][1] for w in waypoints]
    
    for ex_id, _ in enumerate(tqdm.tqdm(dfs)):
        if 'current_node' not in dfs[ex_id].columns:
            logging.warn(f'no cell found in {db3_files[ex_id]}')
            continue

        # find the fitting directory name to give a good name to the experiment
        name = dfs[ex_id].db3.iloc[0].split('/')[-2]
        dfs[ex_id]['experiment'] = str(name)
        
        dfs[ex_id]['robot_id'] = ""
        dfs[ex_id]['pair_id'] = ""
        # TODO: provide this for robots without position
        for robot in dfs[ex_id].robot.unique():
            if pd.isna(robot):
                continue
            if 'x' not in dfs[ex_id].columns:
                # if no names are provided, assume robot names are ordered i.e. robotA, robotB, robotC
                rid = dfs[ex_id].robot.unique().tolist().index(robot)
            elif not len(dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'x']):
                print(f'robot {robot} not found in {ex_id}')
                continue
            else:
                x = dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'x'].iloc[0]
                y = dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'y'].iloc[0]
                rid = get_robot_id(x, y, starting_positions)
            # find the starting position for each tb and assign a name
            dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'robot_id'] = f'robot{rid+1}'
            # fint the pair by the starting position
            dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'pair_id'] = f'pair{int(rid / 2)+1}'

        if 'x' not in dfs[ex_id].columns:
            dfs[ex_id]['x'] = 0.0
            dfs[ex_id]['y'] = 0.0
            dfs[ex_id]['theta'] = 0.0
            dfs[ex_id]['rot_vel'] = 0.0
            dfs[ex_id]['trans_vel'] = 0.0
            dfs[ex_id]['cmd_vel_rot'] = 0.0
            dfs[ex_id]['cmd_vel_trans'] = 0.0
        #dfs[ex_id]['cell'] = dfs[ex_id].apply(find_cell, axis=1)

        dfs[ex_id].rename(columns={'current_node': 'cell'}, inplace=True)
        #dfs[ex_id].loc[dfs[ex_id].cell == dfs[ex_id].cell.shift(), "cell"] = pd.NA
        dfs[ex_id]['cell'] = dfs[ex_id].cell.astype("Int64")

    return dfs


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Aggregates the turtlebot data to one table',

    )
    parser.add_argument('directory', type=str, help='path to the directory that holds the rosbag files')
    parser.add_argument('-o', '--out', metavar='output_file', default=None, type=str, help='path to the output pkl file')
    parser.add_argument('-p', type=int, default=8, help='number of processes to use')
    parser.add_argument('--no-cache', action='store_true', help='do not use the cached data for the db3 files')
    parser.add_argument('-v', '--verbose', action='store_true', help='enable verbose logging (sets log level to INFO)')
    parser.add_argument('--waypoints-file', metavar='waypoints_file', default=None, type=str, help='path to the waypoints file')
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.INFO)

    db3 = get_db3_files_in_folders(args.directory)
    logging.info(f'found {len(db3)} db3 files' )
    
    with Pool(args.p) as pool:
        dfs = pool.map(partial(aggregate_file, use_cached=not args.no_cache), db3)
    logging.info(f'found {len(dfs)} dfs' )
    dfs = data_assignments(dfs, db3, pos_file=args.waypoints_file)
    df = pd.concat(dfs)
    df.robot_id = df.robot_id.astype("category")
    df.pair_id = df.pair_id.astype("category")
    df.experiment = df.experiment.astype("category")
    if args.out is None:
        args.out = args.directory + '/data.prq'
    df.to_parquet(args.out)

