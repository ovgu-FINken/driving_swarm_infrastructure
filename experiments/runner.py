#!/usr/bin/python3

import sys
import os
import subprocess
import time
import argparse
import yaml
import logging
import glob

def execute(cmd, run_dir="", timeout=float('inf')):
    t = time.time()
    timed_out = False
    logging.info(f'running command: \n{cmd}')
    popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True, cwd=run_dir)
    for stdout_line in iter(popen.stdout.readline, ""):
        if time.time() - t > timeout:
            timed_out = True
            break
        yield stdout_line
    popen.stdout.close()
    if timed_out:
        logging.warning("timeout executing command: \n%s" % (cmd))
        popen.kill()
    return_code = popen.wait()
    return return_code
    

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
    parser.add_argument("--print-ros", action="store_true", help="print ros logging output to stdout")
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
                               for n in range(1, config["n_robots"]+1)
                               for algo in algorithms
                               for mode in config["modes"]
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
            csv_file = glob.glob(os.path.join(run_dir, "run_*.csv.gz"))
            if len(csv_file):
                logging.info(f"run {csv_file} already exists")
                continue
        else:
            os.makedirs(run_dir)

        algo_params = run_cfg["algo"][1]
        param_list = [f"{k}:={v}" for k, v in algo_params.items()]


        
        # launch the code
        command = [
            "ros2", 
            "launch",
            "ccr",
            f"ccr_{run_cfg['mode']}.launch.py",
            f"ccr_version:={run_cfg['algo'][0]}",
            f"n_robots:={run_cfg['n']}",
            f"waypoints_file:={config['waypoints_file']}",
            f"run_timeout:={config['run_timeout']:.1f}",
            f"init_timeout:={config['init_timeout']:.1f}",
            "use_rviz:=false",
            "use_rosbag:=false",
            "simulator:=gzserver",
            f"data_file:={run_dir}/run_{run_cfg['algo'][0]}_{run_cfg['mode']}_{run_cfg['n']}_{run_cfg['run']}.csv.gz",
        ]
        command += param_list
        logging.info(f"===================\n{run_cfg_to_str(run_cfg)}\n===================")
        logging.info(f"===================\n{' '.join(command)}\n===================")
        if args.check:
            continue

        with open(os.path.join(run_dir, "params.yaml"), 'w') as f:
            run_params = run_cfg.copy()
            run_params["algorithm"] = run_params["algo"][0]
            run_params["algorithm_params"] = run_params["algo"][1]
            del run_params["algo"]
            yaml.dump(run_params, f)

        with open(os.path.join(run_dir, "ros.log"), 'w', buffering=8*1024) as f:
            # estimate real time factor
            rtf = 2 * config["n_robots"]
            # estimate runtime as rtf * (run_timeout + init_timeout)
            timeout = rtf * (config["run_timeout"]+config["init_timeout"])
            for output in execute(command, run_dir=run_dir, timeout=timeout):
                if args.print_ros:
                    print(output, end="")
                f.write(output)

        logging.info(f"done config {run_cfg}")

if __name__ == "__main__":
    main()
