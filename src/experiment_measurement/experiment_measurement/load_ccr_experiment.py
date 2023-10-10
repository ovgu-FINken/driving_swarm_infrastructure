from multiprocessing.pool import Pool
import os
import argparse
import pandas.io.sql as pdsql
import yaml
import numpy as np
import pandas as pd
import tqdm
from data_export import read_rosbag_all_in_one,  DataConverter
from glob import glob

def get_db3_files_in_folders(directory):
    return glob(f'{directory}/**/*.db3', recursive=True)

def aggregate_file(db3_file):
    if os.path.isfile(db3_file.replace('.db3', '.pkl')):
        return pd.read_pickle(db3_file.replace('.db3', '.pkl'))
    try:
        print(f'start aggregating {db3_file}')
        topics = ["%goal_completed", "/tf", "/clock", "/command", "%status", "%cmd_vel", "%current_node"]
        data = read_rosbag_all_in_one(db3_file, topics=topics)
        print(f'table aggregating {db3_file}')
        converter = DataConverter()
        df = converter.convert_df(data)
        df['db3'] = db3_file
        df.to_pickle(db3_file.replace('.db3', '.pkl'))
        print(f'done  aggregating {db3_file}')
        return df
    except pdsql.DatabaseError:
        print(f'error aggregating {db3_file}')
        return pd.DataFrame()

def get_robot_id(x, y, start_pos):
    dist = [np.linalg.norm(np.array([x-p[0], y-p[1]])) for p in start_pos]
    return int(np.argmin(np.array(dist)))

def get_type_from_name(name):
    if "real" in name:
        return "real"
    if "sim" in name:
        return "simulated"
    return "UNCLASSIFIED"

def get_algo_from_name(name):
    if "baseline" in name:
        return "baseline"
    if "fixed_priorities" in name:
        return "fixed_priorities"
    if "same_priorities" in name:
        return "same_priorities"
    if "ccr" in name:
        return "ccr"
    return "UNCLASSIFIED"


# because we switched turtlebots during experiments we have to assign new names and pairs by the robots starting position
def data_assignments(dfs, db3_files):
    
    pos_file = '/home/semai/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/params/icra2024_waypoints1m.yaml'
    with open(pos_file, 'r') as file:
        waypoints = yaml.safe_load(file)
    starting_positions = [w['waypoints'][1] for w in waypoints]
    
    for ex_id, _ in enumerate(tqdm.tqdm(dfs)):
        # find the fitting directory name to give a good name to the experiment
        name = dfs[ex_id].db3.iloc[0].split('/')[-2]
        dfs[ex_id]['experiment'] = str(name)
        dfs[ex_id]['type'] = dfs[ex_id].db3.apply(get_type_from_name)
        dfs[ex_id]['algo'] = dfs[ex_id].db3.apply(get_algo_from_name)
        
        dfs[ex_id]['robot_id'] = ""
        dfs[ex_id]['pair_id'] = ""
        for robot in dfs[ex_id].robot.unique():
            if pd.isna(robot):
                continue
            if not len(dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'x']):
                print(f'robot {robot} not found in {ex_id}')
                continue
            x = dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'x'].iloc[0]
            y = dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'y'].iloc[0]
            rid = get_robot_id(x, y, starting_positions)
            # find the starting position for each tb and assign a name
            dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'robot_id'] = f'robot{rid+1}'
            # fint the pair by the starting position
            dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'pair_id'] = f'pair{int(rid / 2)+1}'

        #dfs[ex_id]['cell'] = dfs[ex_id].apply(find_cell, axis=1)
        dfs[ex_id].rename(columns={'current_node': 'cell'}, inplace=True)
        #dfs[ex_id].loc[dfs[ex_id].cell == dfs[ex_id].cell.shift(), "cell"] = pd.NA
        dfs[ex_id]['cell'] = dfs[ex_id].cell.astype("Int64")
        dfs[ex_id]['N'] = len(dfs[ex_id].loc[~dfs[ex_id].robot_id.isna()].robot_id.unique())

    return dfs


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Aggregates the turtlebot data to one table',

    )
    parser.add_argument('directory', type=str, help='path to the directory that holds the rosbag files')
    parser.add_argument('-o', '--out', metavar='output_file', default="data.pkl", type=str, help='path to the output pkl file')
    args = parser.parse_args()

    db3 = get_db3_files_in_folders(args.directory)
    print(f'found {len(db3)} db3 files' )
    
    with Pool(8) as pool:
        dfs = pool.map(aggregate_file, db3)
    print(f'found {len(dfs)} dfs' )
    dfs = data_assignments(dfs, db3)
    df = pd.concat(dfs)
    df.robot_id = df.robot_id.astype("category")
    df.pair_id = df.pair_id.astype("category")
    df.experiment = df.experiment.astype("category")
    df.to_pickle(args.out)

