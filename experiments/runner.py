#!/usr/bin/python3

import sys
import os
import subprocess
import time
import argparse
import yaml
import logging
import glob


### MAKE a YAML file, which describes the target experiment settings
# FORMAT:
# - N_ROBOTS: number of robots
# - N_RUNS: number of runs
# - RUN_TIMEOUT: timeout for each run
# - INIT_TIMEOUT: timeout for each init
# - Algorithm
# - Mode (FAKE/SIM)
# - waypoints_file
# - output_dir

def main():
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument("-config_file", type=str, default=None)
    parser.add_argument("-n_robots", type=int, default=None)
    parser.add_argument("-n_runs", type=int, default=None)
    parser.add_argument("-run_timeout", type=float, default=None)
    parser.add_argument("-init_timeout", type=float, default=None)
    parser.add_argument("-algorithms", nargs='+', type=str, default=[])
    parser.add_argument("-modes", nargs='+', type=str, default=[])
    parser.add_argument("-waypoints_file", type=str, default=None)
    parser.add_argument("-output_dir", type=str, default=f"experiment_{time.strftime('%Y-%m-%d')}")
    parser.add_argument("--check", action="store_true")
    parser.add_argument('--only-max-agents', action='store_true', help='only use the maximum number of agents per experiment (the default behavior is to iterate over all N=1, ... agents)')
    args = parser.parse_args()

    config = {}
    if args.config_file is None:
        print("No config file")
        
    else:
        with open(args.config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    
    if args.n_robots is not None:
        config["n_robots"] = args.n_robots

    if args.n_runs is not None:
        config["n_runs"] = args.n_runs

    if args.run_timeout is not None:
        config["run_timeout"] = args.run_timeout

    if args.init_timeout is not None:
        config["init_timeout"] = args.init_timeout

    if len(args.algorithms):
        config["algorithms"] = args.algorithms

    if len(args.modes):
        config["modes"] = args.modes

    if args.waypoints_file is not None:
        config["waypoints_file"] = args.waypoints_file


    config["output_dir"] = args.output_dir

    # create output directory:
    if not os.path.exists(config["output_dir"]):
        os.makedirs(config["output_dir"])
    else:
        logging.info(f"output directory already exists: {config['output_dir']}")

    with open(os.path.join(config["output_dir"], "config.yaml"), 'w') as f:
        logging.info("writing config.yaml")
        yaml.dump(config, f)


    # each algorithm contains a dict and parameters with a list of possible values
    # here we create the crossproduct of all parameter combinations, such that each algorithm is later run with all listed parameter combinations
    
    algorithms = []
    for algorithm, params in config["algorithms"].items():
        # create cross product of all parameter combinations
        if not len(params):
            algorithms.append( (algorithm, {}) )
            continue

        # create cross product of all parameter combinations
        product = [{}]
        for param, values in params.items():
            print(f"param: {param}, values: {values}")
            product = [p | {param: v} for p in product for v in values]
        for p in product:
            algorithms.append( (algorithm, p) )
    print(algorithms)
        
    # run experiments
    # create list of configurations:
    if args.only_max_agents:
        run_configurations = [{"algo": algo, "mode": mode, "n": config["n_robots"], "run": run} \
                               for algo in algorithms
                               for mode in config["modes"]
                               for run in range(1, config["n_runs"]+1)
                            ]
    else: 
        run_configurations = [{"algo": algo, "mode": mode, "n": n, "run": run} \
                               for algo in algorithms
                               for mode in config["modes"]
                               for n in range(1, config["n_robots"]+1)
                               for run in range(1, config["n_runs"]+1)
                            ]
    
    def run_cfg_to_str(run_cfg):
        params = "_".join([f"{k}={v}" for k, v in run_cfg["algo"][1].items()])
        return run_cfg["algo"][0] + "_" + params + \
        f"_{run_cfg['mode']}_{run_cfg['n']}_{run_cfg['run']}"

    for run_cfg in run_configurations:
        # check if run directory exists
        run_dir = os.path.join(config["output_dir"], run_cfg_to_str(run_cfg))
        if os.path.exists(run_dir):
            logging.info("run directory already exists")
            # check if db3 file exists in run directory:
            # the db3 file is within a rosbag_... directory in the run directory
            db3_file = glob.glob(os.path.join(run_dir, "rosbag_*/*db3"))
            if len(db3_file):
                logging.info("run db3 already exists")
                continue
            ### TODO: check if run seem legit (file size > threshold)
        # run_dir does not exist
        else:
            os.makedirs(run_dir)

        # execute command: ros2 launch ccr ccr_$MODE.launch.py use_rviz:=false use_rosbag:=true n_robots:=$N_ROBOTS waypoints_file:=$MAP
        command = [
            "ros2", 
            "launch",
            "ccr",
            f"ccr_{run_cfg['mode']}.launch.py",
            "use_rviz:=false",
            "use_rosbag:=true",
            f"n_robots:={run_cfg['n']}",
            f"waypoints_file:={config['waypoints_file']}",
            f"run_timeout:={config['run_timeout']:.1f}",
            f"init_timeout:={config['init_timeout']:.1f}",
        ]
        logging.info(f"===================\n{run_cfg_to_str(run_cfg)}\n===================")
        logging.info(f"===================\n{' '.join(command)}\n===================")
        if args.check:
            continue
        try:
            # save parameter settings for this run as yaml in the run directory
            with open(os.path.join(run_dir, "params.yaml"), 'w') as f:
                run_params = run_cfg.copy()
                run_params["algorithm"] = run_params["algo"][0]
                run_params["algorithm_params"] = run_params["algo"][1]
                del run_params["algo"]
                yaml.dump(run_params, f)

            # we calculate with a realtime factor should be better than 0.1
            output = ""
            process = subprocess.run(command,
                                capture_output=True,
                                cwd=run_dir,
                                timeout=config["run_timeout"] * 10 +
                                config["init_timeout"] * 10
                                )
            output = process.stdout.decode("utf-8")
            err = process.stderr.decode("utf-8")
            with open(os.path.join(run_dir, "output.log"), 'w') as f:
                f.write(err)
                f.write("\n\n\n")
                f.write(output)
        except subprocess.TimeoutExpired:
            output = process.stdout.decode("utf-8")
            err = process.stderr.decode("utf-8")
            with open(os.path.join(run_dir, "output.log"), 'w') as f:
                f.write(err)
                f.write("\n\n\n")
                f.write(output)
            logging.warning(f"timeout for config {run_cfg}")
        except KeyboardInterrupt:
            logging.warning(f"keyboard interrupt for config {run_cfg}")
            with open(os.path.join(run_dir, "output.log"), 'w') as f:
                f.write(err)
                f.write("\n\n\n")
                f.write(output)
            logging.warning(f"timeout for config {run_cfg}")


        logging.info(f"done config {run_cfg}")

if __name__ == "__main__":
    main()
