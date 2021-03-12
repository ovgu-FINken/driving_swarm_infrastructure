"""Aggregate the data to one table."""
from experiment_measurement import data_aggregation_helper
import numpy as np


"""
table_column_config  ::= table_column, [ df_aggregated_topics ]
table_column         ::= ( topic, table_column_name, func )

topic                ::= string
table_column_name    ::= string
func                 ::= message_dataframe, time -> Object

message_dataframe    ::= pd.Series      # All messages to one topic and robot
time                 ::= number
"""
table_column_config = [
    data_aggregation_helper.TableColumn(
        'cmd_vel',
        'trans_vel',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)[
            'data'
        ].linear.x,
    ),
    data_aggregation_helper.TableColumn(
        'cmd_vel',
        'rot_vel',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)[
            'data'
        ].angular.z,
    ),
    data_aggregation_helper.TableColumn(
        'amcl_pose',
        'pose_x',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)[
            'data'
        ].pose.pose.position.x,
    ),
    data_aggregation_helper.TableColumn(
        'amcl_pose',
        'pose_y',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)[
            'data'
        ].pose.pose.position.y,
    ),
    data_aggregation_helper.TableColumn(
        'amcl_pose',
        'pose_theta',
        lambda conf: data_aggregation_helper.quaternion_to_euler(
            data_aggregation_helper.get_latest_in_interval(conf)['data']
        )[2],
    ),
    data_aggregation_helper.TableColumn(
        'amcl_pose',
        'amcl_traveled_distance',
        lambda conf: data_aggregation_helper.calculate_travelled_distance(
            data_aggregation_helper.amcl_calc_x_and_y(conf)
        ),
    ),
    data_aggregation_helper.TableColumn(
        'scan',
        'min_obstacle_dist',
        lambda conf: min(
            data_aggregation_helper.get_latest_in_interval(conf)['data'].ranges
        ),
    ),
    data_aggregation_helper.TableColumn(
        'measurement/n_neighbors',
        'n_neighbors',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    ),
    data_aggregation_helper.TableColumn(
        'measurement/target',
        'target_x',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].x
    ),
    data_aggregation_helper.TableColumn(
        'measurement/target',
        'target_y',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].y
    ),
    data_aggregation_helper.TableColumn(
        'measurement/tf_error',
        'tf_error',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    ),
    # data_aggregation_helper.TableColumn(
    #     'inter_robot_dist',
    #     'inter_robot_dist',
    #     lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    # ),
    data_aggregation_helper.TableColumn(
        'measurement/inter_robot_dist',
        'min_inter_robot_dist',
        lambda conf: min(data_aggregation_helper.get_latest_in_interval(conf)['data'].data),
    ),
    data_aggregation_helper.TableColumn(
        'measurement/inter_robot_dist',
        'max_inter_robot_dist',
        lambda conf: max(data_aggregation_helper.get_latest_in_interval(conf)['data'].data),
    ),    
    # data_aggregation_helper.TableColumn(
    #     'measurement/inter_robot_dist',
    #     'avg_inter_robot_dist',
    #     lambda conf: np.mean(data_aggregation_helper.get_latest_in_interval(conf)['data'].data),
    # ),
    # data_aggregation_helper.TableColumn(
    #     'measurement/forces',
    #     'forces',
    #     lambda conf: np.mean(data_aggregation_helper.get_latest_in_interval(conf)['data'].data),
    # ),
    data_aggregation_helper.TableColumn(
        '/tf',
        'tf_pose_x',
        lambda conf: data_aggregation_helper.get_latest_in_interval(
            data_aggregation_helper.filter_tf_child_frame_id(conf)
        )[
            'data'
        ].transforms[0].transform.translation.x,
    ),
    data_aggregation_helper.TableColumn(
        '/tf',
        'tf_pose_y',
        lambda conf: data_aggregation_helper.get_latest_in_interval(
            data_aggregation_helper.filter_tf_child_frame_id(conf)
        )[
            'data'
        ].transforms[0].transform.translation.y,
    ),
    data_aggregation_helper.TableColumn(
        '/tf',
        'tf_pose_theta',
        lambda conf: data_aggregation_helper.quaternion_to_euler(
            data_aggregation_helper.get_latest_in_interval(
                data_aggregation_helper.filter_tf_child_frame_id(conf)
            )['data'].transforms[0].transform.rotation
        )[2],
    ),
    data_aggregation_helper.TableColumn(
        '/tf',
        'tf_traveled_distance',
        lambda conf: data_aggregation_helper.calculate_travelled_distance(
            data_aggregation_helper.tf_calc_x_and_y(
                data_aggregation_helper.filter_tf_child_frame_id(conf)
            )
        ),
    ),]
