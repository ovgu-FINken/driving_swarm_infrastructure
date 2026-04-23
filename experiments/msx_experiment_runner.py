#!/usr/bin/env python3

import argparse
import itertools
import os
import subprocess
import time
from datetime import datetime
import yaml
import signal
import pandas as pd

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

    likelihoods = param_dict["likelihood_function"]

    always = [
        "debug",
        "use_bayes_filter",
        "dist_threshold",
        "angle_threshold",
        "scale_error_weight",
        "angle_error_weight",
        "likelihood_clamp",
        "old_error_penalty_scale",
    ]

    for lf in likelihoods:

        lf_norm = normalize(lf)

        required = LIKELIHOOD_SCHEMA[lf_norm]

        keys = always + ["likelihood_function"] + required
        values = [param_dict[k] for k in always] + [[lf]] + [param_dict[k] for k in required]

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
    v = normalize(value) if isinstance(value, str) else value
    return f"{key}__{sanitize(v)}"


def get_multi_value_params(param_grid):
    """Returns param names with >1 value in YAML order."""
    return [k for k, v in param_grid.items()
            if isinstance(v, list) and len(v) > 1]


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

def verify_moved(csv_path="../data.csv.gz", robot="robotA", x_column="x", threshold=0.01):
    """
    Checks whether the x-position of a single robot changed enough
    to be considered movement.

    The CSV is expected to contain a robot identifier column
    (e.g. values like robotA, robotB, ...).

    Args:
        csv_path (str): Path to the .csv.gz file
        robot (str): Robot name to filter for (e.g. "robotA")
        x_column (str): Name of the x-position column
        threshold (float): Minimum required change (max - min)

    Returns:
        True  -> movement detected
        False -> no relevant movement / error / insufficient data
    """
    if not os.path.exists(csv_path):
        print("Running Error: path does not exist")
        return False

    try:
        df = pd.read_csv(csv_path, compression="gzip")

        if "robot" not in df.columns:
            print("Running Error: no robots")
            return False

        if x_column not in df.columns:
            print("Running Error: no x column")
            return False

        robot_df = df[df["robot"] == robot]

        if robot_df.empty:
            print("Running Error: empty df")
            return False

        values = robot_df[x_column].dropna()

        if len(values) < 2:
            print("Running Error: <2 values")
            return False

        movement = values.max() - values.min()

        return movement > threshold

    except Exception as e:
        print(f"verify_moved() error: {e}")
        return False

# =========================================================
# PARAMETERS
# =========================================================

def get_active_parameters(combo):
    lf = normalize(combo["likelihood_function"])

    params = [("likelihood_function", lf)]

    for key in LIKELIHOOD_SCHEMA[lf]:
        params.append((key, combo[key]))

    return params

def get_relevant_parameters(combo):
    lf = normalize(combo["likelihood_function"])

    always = [
        "debug",
        "use_bayes_filter",
        "dist_threshold",
        "angle_threshold",
        "scale_error_weight",
        "angle_error_weight",
        "likelihood_function",
        "likelihood_clamp",
        "old_error_penalty_scale",
    ]

    relevant = always + LIKELIHOOD_SCHEMA[lf]

    return {k: combo[k] for k in relevant if k in combo}

# =========================================================
# DIRECTORY STRUCTURE
# =========================================================

def build_parameter_directory(base_dir, n_robots, combo, multi_value_params):

    validate_combo(combo)

    # n_robots is always a directory level
    path = [base_dir, f"n_robots__{n_robots}"]

    for key in multi_value_params:
        if key not in combo:
            continue
        if key == "likelihood_function":
            # use just the normalized value (e.g. linear_clamped), no key prefix
            path.append(normalize(combo[key]))
        else:
            path.append(param_dir(key, combo[key]))

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

def run_command(cmd, timeout, test_mode, headless_mode, run_dir, run_id):

    print("\n==============================")
    print(cmd)
    print("==============================")

    if test_mode:
        print("[TEST MODE]")
        return

    env = os.environ.copy()

    if headless_mode:
        env["ROS_SIMULATOR"] = "gzserver"
    else:
        env["ROS_SIMULATOR"] = "gazebo"

    proc = subprocess.Popen(cmd, preexec_fn=os.setsid, env=env)

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
    parser.add_argument("-hl", "--headless", action="store_true")
    args = parser.parse_args()

    cfg = load_yaml(CONFIG_YAML)

    runs = cfg["runs"]
    min_robots = cfg.get("min_robots", 1)
    max_robots = cfg["max_robots"]
    param_grid = cfg["parameters"]
    run_timeout = cfg.get("runtime_seconds", 60)

    multi_value_params = get_multi_value_params(param_grid)

    combos = list(build_combinations(param_grid))

    # validate ALL combinations early (important)
    for c in combos:
        validate_combo(c)

    os.makedirs(BASE_DIR, exist_ok=True)

    num_combos = len(combos)
    num_robot_configs = max_robots - min_robots + 1

    total_experiments = runs * num_robot_configs * num_combos
    est_seconds = total_experiments * (run_timeout + KILL_BUFFER)

    plan_text = (
        "======================================\n"
        "EXPERIMENT PLAN\n"
        "======================================\n"
        f"Runs: {runs}\n"
        f"Robot configs: {num_robot_configs}\n"
        f"Combinations: {num_combos}\n"
        f"Total experiments: {total_experiments}\n"
        f"Estimated time: {est_seconds/60:.1f} min ({est_seconds/3600:.2f} h)\n"
        "======================================\n"
)
    print("\n" + plan_text + "\n")

    with open(os.path.join(BASE_DIR, "experiment_plan.txt"), "w") as f:
        f.write(plan_text + "\n")

    # =====================================================
    # EXECUTION LOOP
    # =====================================================

    for n_robots in range(min_robots, max_robots + 1):

        for combo in combos:

            run_dir = build_parameter_directory(BASE_DIR, n_robots, combo, multi_value_params)
            os.makedirs(run_dir, exist_ok=True)

            # save full config snapshot
            param_file = os.path.join(run_dir, "params.txt")


            if not os.path.exists(param_file):
                with open(param_file, "w") as f:
                    f.write(f"n_robots: {n_robots}\n")
                    f.write(f"runtime_seconds: {run_timeout}\n\n")

                    params = get_relevant_parameters(combo)

                    for k, v in params.items():
                        f.write(f"{k}: {v}\n")

            #print(f"COMBO : {build_ros_args(combo)}")

            run_id = 1

            while run_id <= runs:
                cmd = [
                    "ros2",
                    "launch",
                    "msx",
                    "msx.launch.py",
                    "use_rviz:=false",
                    f"n_robots:={n_robots}",
                    *build_ros_args(combo),
                ]

                run_command(cmd, run_timeout, args.test, args.headless, run_dir, run_id)

                # sleep to wait for data to be written (shouldn't be neccessary but anyways...)
                time.sleep(1)

                last_file = os.path.join(run_dir, f"run_{run_id}.csv.gz")

                moved = verify_moved(last_file)

                if moved:
                    print(f"Run {run_id}: movement detected -> accepted")
                    run_id += 1
                else:
                    print(f"Run {run_id}: no movement detected -> repeating same run")

    print("\n" + plan_text + "\n")

if __name__ == "__main__":
    main()
