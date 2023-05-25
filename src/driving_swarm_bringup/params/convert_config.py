import argparse
import yaml


def read_yaml_file(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)
    
def write_yaml_file(file_path, data):
    with open(file_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert config file from old format to new format, i.e. seperate files for poses, goals, and robot names')
    parser.add_argument('-input_file', help='Input file')
    parser.add_argument('-output_prefix', help='Output file')
    
    args = parser.parse_args()

    input_config = read_yaml_file(args.input_file)
    output_poses = []
    output_goals = []
    for i, robot in enumerate(input_config):
        output_poses.append([robot['x_pose'], robot['y_pose'], robot['yaw_pose']])
        if 'goals' not in robot:
            continue
        goals = [ [x, y, yaw] for x, y, yaw in zip(robot['goals']['x'], robot['goals']['y'], robot['goals']['theta']) ]
        output_goals.append({ 'waypoints' : goals})
    
    write_yaml_file(args.output_prefix + '_poses.yaml', output_poses)
    write_yaml_file(args.output_prefix + '_waypoints.yaml', output_goals)
    


