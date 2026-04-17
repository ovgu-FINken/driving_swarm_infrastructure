#!/usr/bin/env python3

import argparse
import itertools
import os
import subprocess
import time
from datetime import datetime
import yaml
import signal

# =========================================================
# CONFIG
# =========================================================

CONFIG_YAML = "msx_experiments.yaml"
KILL_BUFFER = 10

TIMESTAMP = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
BASE_DIR = f"./experiment_{TIMESTAMP}"


# =========================================================
# VALIDATION SCHEMA
# =========================================================

LIKELIHOOD_SCHEMA = {
    "linear_clamped": ["a_1"],
    "negative_log_clamped": ["a_2"],
    "gauss_clamped": ["a_3", "m", "s"],
}


# =========================================================
# LOAD YAML
# =========================================================

def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


# =========================================================
# COMBINATIONS
# =========================================================

def build_combinations(param_dict):
    keys = list(param_dict.keys())
    values = [param_dict[k] for k in keys]

    for combo in itertools.product(*values):
        yield dict(zip(keys, combo))


def build_ros_args(param_dict):
    return [f"{k}:={v}" for k, v in param_dict.items()]


# =========================================================
# UTILS
# =========================================================

def normalize(v):
    return str(v).strip().lower()


def sanitize(v):
    return str(v).replace(".", "_").replace("-", "neg")


def param_dir(key, value):
    return f"{key}__{sanitize(value)}"


# =========================================================
# VALIDATION
# =========================================================

def validate_combo(combo):
    """
    Ensures:
    - likelihood_function exists
    - correct name
    - required parameters exist
    """

    if "likelihood_function" not in combo:
        raise ValueError("Missing likelihood_function")

    lf = normalize(combo["likelihood_function"])

    if lf not in LIKELIHOOD_SCHEMA:
        raise ValueError(
            f"Unknown likelihood_function: {lf}. "
            f"Valid: {list(LIKELIHOOD_SCHEMA.keys())}"
        )

    missing = [p for p in LIKELIHOOD_SCHEMA[lf] if p not in combo]

    if missing:
        raise ValueError(
            f"Missing parameters for {lf}: {missing}"
        )

    return lf


# =========================================================
# ACTIVE PARAMETERS
# =========================================================

def get_active_parameters(combo):
    lf = normalize(combo["likelihood_function"])

    params = [("likelihood_function", lf)]

    for key in LIKELIHOOD_SCHEMA[lf]:
        params.append((key, combo[key]))

    return params


# =========================================================
# DIRECTORY STRUCTURE
# =========================================================

def build_parameter_directory(base_dir, n_robots, combo):

    lf = validate_combo(combo)

    params = get_active_parameters(combo)

    path = [
        base_dir,
        f"n_robots__{n_robots}",
        lf,
    ]

    for key, value in params[1:]:
        path.append(param_dir(key, value))

    full_path = os.path.join(*path)

    print("[DIR]", full_path)

    return full_path


# =========================================================
# PROCESS CONTROL
# =========================================================

def wait_for_process_exit(proc, timeout):
    start = time.time()

    while time.time() - start < timeout:
        if proc.poll() is not None:
            return True
        time.sleep(1)

    return False


def wait_for_file_stable(path, timeout=300, stable_time=3):
    start = time.time()
    last_size = -1
    stable = 0

    while time.time() - start < timeout:

        if os.path.exists(path):
            size = os.path.getsize(path)

            if size == last_size and size > 0:
                stable += 1
            else:
                stable = 0
                last_size = size

            if stable >= stable_time:
                return True

        time.sleep(1)

    return False


def kill_gazebo_if_stuck(timeout=10):
    start = time.time()

    while time.time() - start < timeout:
        alive = []

        for name in ["gzserver", "gzclient"]:
            try:
                out = subprocess.check_output(["pgrep", "-f", name]).decode().strip()
                if out:
                    alive.extend(out.splitlines())
            except subprocess.CalledProcessError:
                pass

        if not alive:
            return

        time.sleep(1)

    subprocess.run(["pkill", "-SIGKILL", "-f", "gzserver"])
    subprocess.run(["pkill", "-SIGKILL", "-f", "gzclient"])


# =========================================================
# RUN
# =========================================================

def run_command(cmd, timeout, test_mode, run_dir, run_id):

    print("\n==============================")
    print(cmd)
    print("==============================")

    if test_mode:
        print("[TEST MODE]")
        return

    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

    try:
        time.sleep(timeout)

        os.killpg(os.getpgid(proc.pid), signal.SIGINT)

        wait_for_process_exit(proc, KILL_BUFFER)
        kill_gazebo_if_stuck()

    finally:
        src = "data.csv.gz"

        if wait_for_file_stable(src) and os.path.exists(src):

            dst = os.path.join(run_dir, f"run_{run_id}.csv.gz")
            subprocess.run(["mv", src, dst])

        else:
            print("[WARN] missing output file")


# =========================================================
# MAIN
# =========================================================

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--test", action="store_true")
    args = parser.parse_args()

    start_time = time.time()

    cfg = load_yaml(CONFIG_YAML)

    runs = cfg["runs"]
    min_robots = cfg.get("min_robots", 1)
    max_robots = cfg["max_robots"]
    param_grid = cfg["parameters"]
    run_timeout = cfg.get("runtime_seconds", 60)

    combos = list(build_combinations(param_grid))

    # validate ALL combinations early (important)
    for c in combos:
        validate_combo(c)

    os.makedirs(BASE_DIR, exist_ok=True)

    num_combos = len(combos)
    num_robot_configs = max_robots - min_robots + 1

    total_experiments = runs * num_robot_configs * num_combos
    est_seconds = total_experiments * (run_timeout + KILL_BUFFER)

    print("\n======================================")
    print("EXPERIMENT PLAN")
    print("======================================")
    print(f"Runs: {runs}")
    print(f"Robot configs: {num_robot_configs}")
    print(f"Combinations: {num_combos}")
    print(f"Total experiments: {total_experiments}")
    print(f"Estimated time: {est_seconds/60:.1f} min ({est_seconds/3600:.2f} h)")
    print("======================================\n")

    # =====================================================
    # EXECUTION LOOP
    # =====================================================

    for n_robots in range(min_robots, max_robots + 1):

        for combo in combos:

            run_dir = build_parameter_directory(BASE_DIR, n_robots, combo)
            os.makedirs(run_dir, exist_ok=True)

            # save full config snapshot
            param_file = os.path.join(run_dir, "params.txt")

            if not os.path.exists(param_file):
                with open(param_file, "w") as f:
                    f.write(f"n_robots: {n_robots}\n")
                    f.write(f"runtime_seconds: {run_timeout}\n\n")
                    for k, v in combo.items():
                        f.write(f"{k}: {v}\n")

            #print(f"COMBO : {build_ros_args(combo)}")

            for run_id in range(1, runs + 1):

                cmd = [
                    "ros2",
                    "launch",
                    "msx",
                    "msx.launch.py",
                    "use_rviz:=false",
                    f"n_robots:={n_robots}",
                    *build_ros_args(combo),
                ]

                run_command(cmd, run_timeout, args.test, run_dir, run_id)


if __name__ == "__main__":
    main()
