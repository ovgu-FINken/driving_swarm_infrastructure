from multiprocessing.pool import Pool
import os
import argparse
import pandas.io.sql as pdsql
import yaml
import numpy as np
import pandas as pd
import tqdm
from polygonal_roadmaps import environment, geometry
from data_export import read_rosbag_all_in_one,  DataConverter

def get_db3_files_in_folders(directory):
    db3_files = {}
    # Iterate through immediate subdirectories
    for subdir in next(os.walk(directory))[1]:
        subdir_path = os.path.join(directory, subdir)

        # Iterate through files in the subdirectory
        for filename in os.listdir(subdir_path):
            if filename.endswith(".db3"):
                db3_files[subdir] = os.path.join(subdir_path, filename)

    return db3_files

def aggregate_file(db3_file, config_file, step_size):
    if os.path.isfile(db3_file.replace('.db3', '.pkl')):
        return pd.read_pickle(db3_file.replace('.db3', '.pkl'))
    try:
        print(f'start aggregating {db3_file}')
        data = read_rosbag_all_in_one(db3_file)
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
    return "ccr"

class map_loader_without_ros:
    def __init__(self, graph_file=None):
        # Simulating parameter declarations and assignments without ROS
        self.parameters = {
        'graph_file': graph_file,
        'x_min': -2.25,
        'x_max': 2.75,
        'y_min': -1.75,
        'y_max': 1.25,
        'grid_type': 'square',
        'grid_size': 0.5,
        'inflation_size': 0.1,
        'laser_inflation_size': 0.15,
        'vehicle_model': 3,
        'step_size': 0.1,
        'turn_speed': 0.5,
        'inertia': 0.01,
        'belief_lifetime': 15.0,
        'belief_lifetime_variability': 2.0,
        'horizon': 5,
        'wait_cost': 1.01,
            
        }

        map_file = self.get_parameter('graph_file')
        print(f"map_file value: {map_file}")  # print the value of map_file

        if map_file.endswith(".yaml"):
            grid_size = self.get_parameter('grid_size')
            tiling = self.get_parameter('grid_type')
            print(f"tiling value: {tiling}")  # print the value of tiling
            
            wx = (self.get_parameter('x_min'), self.get_parameter('x_max'))
            wy = (self.get_parameter('y_min'), self.get_parameter('y_max'))

            points = None
            if tiling == 'hex':
                points = geometry.hexagon_tiling(grid_size, working_area_x=wx, working_area_y=wy)
            elif tiling == 'square':
                points = geometry.square_tiling(grid_size, working_area_x=wx, working_area_y=wy)
            elif tiling == 'random':
                points = geometry.random_tiling(50, working_area_x=wx, working_area_y=wy)
            else:
                print('no tiling specified, using hex')
                points = geometry.hexagon_tiling(grid_size, working_area_x=wx, working_area_y=wy)

            assert points is not None

            self.env = environment.RoadmapEnvironment(map_file,
                                                       [],
                                                       [],
                                                       generator_points=points,
                                                       wx=wx,
                                                       wy=wy,
                                                       offset=self.get_parameter('inflation_size'))

        else:
            print("map_file does not end with .yaml")

    def get_parameter(self, key):
        return self.parameters.get(key, None)




# because we switched turtlebots during experiments we have to assign new names and pairs by the robots starting position
def data_assignments(dfs, db3_files):
    def find_cell(row):
        return map_obj.env.find_nearest_node((row.x, row.y))
    
    pos_file = '/home/semai/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/params/icra2024_waypoints.yaml'
    map_obj = map_loader_without_ros('/home/semai/ros/driving_swarm_infrastructure/src/driving_swarm_bringup/maps/icra2024.yaml')
    with open(pos_file, 'r') as file:
        waypoints = yaml.safe_load(file)
    starting_positions = [w['waypoints'][1] for w in waypoints]
    
    for ex_id, _ in enumerate(tqdm.tqdm(dfs)):
        # find the fitting directory name to give a good name to the experiment
        name = {v: k for k, v in db3_files.items()}[dfs[ex_id].db3.iloc[0]]
        dfs[ex_id]['experiment'] = name
        dfs[ex_id]['type'] = get_type_from_name(name)
        dfs[ex_id]['algo'] = get_algo_from_name(name)
        
        
        for robot in dfs[ex_id].robot.unique():
            x = dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'x'].iloc[0]
            y = dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'y'].iloc[0]
            rid = get_robot_id(x, y, starting_positions)
            # find the starting position for each tb and assign a name
            dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'robot_id'] = f'robot{rid+1}'
            # fint the pair by the starting position
            dfs[ex_id].loc[dfs[ex_id].robot.eq(robot), 'pair_id'] = f'pair{int(rid / 2)+1}'

        dfs[ex_id]['cell'] = dfs[ex_id].apply(find_cell, axis=1)
        dfs[ex_id].loc[dfs[ex_id].cell == dfs[ex_id].cell.shift(), "cell"] = pd.NA
        dfs[ex_id]['cell'] = dfs[ex_id].cell.astype("Int64")
        dfs[ex_id]['N'] = len(dfs[ex_id].robot_id.unique())

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
    
    inputs = [(f, 'ccr_experiment', 10**9) for f in db3.values()] 
    with Pool(8) as pool:
        dfs = pool.starmap(aggregate_file, inputs)
    print(f'found {len(dfs)} dfs' )
    dfs = data_assignments(dfs, db3)
    df = pd.concat(dfs)
    df.robot_id = df.robot_id.astype("category")
    df.pair_id = df.pair_id.astype("category")
    df.experiment = df.experiment.astype("category")
    df.to_pickle(args.out)

    
