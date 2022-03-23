"""Aggregate the data to one table."""
from experiment_measurement.data_aggregation_helper import *
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
    TableColumn(
        '/tf',
        'tf_pose_x',
        lambda conf: get_latest_in_interval(
            filter_tf_child_frame_id(conf)
        )[
            'data'
        ].transforms[0].transform.translation.x,
    ),
    TableColumn(
        '/tf',
        'tf_pose_y',
        lambda conf: get_latest_in_interval(
            filter_tf_child_frame_id(conf)
        )[
            'data'
        ].transforms[0].transform.translation.y,
    ),
    TableColumn(
        '/tf',
        'tf_pose_theta',
        lambda conf: quaternion_to_euler(
            get_latest_in_interval(  #
                filter_tf_child_frame_id(conf)
            )['data'].transforms[0].transform.rotation
        )[2],
    ),
    TableColumn(
        '/tf',
        'tf_traveled_distance',
        lambda conf: calculate_travelled_distance(
            tf_calc_x_and_y(
                filter_tf_child_frame_id(conf)
            )
        ),
    ),
    # TableColumn(
    #     'scan',
    #     'obstacle_dist',
    #     lambda conf: np.array(
    #         get_latest_in_interval(conf)['data'].ranges
    #     ),
    # ),
    TableColumn(
        'cmd_vel',
        'rot_vel',
        lambda conf: get_earliest_in_next_intervall(conf)[  #
            'data'
        ].angular.z,
    ),
    TableColumn(
        'cmd_vel',
        'trans_vel',
        lambda conf: get_earliest_in_next_intervall(conf)[
            'data'
        ].linear.x,
    ),
    TableColumn(
        'context_steering/vis/params',
        'params',
        lambda conf: get_all_params_as_tuple_list(conf)
    ),
    TableColumn(
        'context_steering/point_cloud',
        'obstacle_dist',
        lambda conf: get_obstacle_distance(
                get_latest_in_interval(conf)['data']
        ),
    ),
    TableColumn(
        'context_steering/samples',
        'samples',
        lambda conf: np.array(
            get_latest_in_interval(conf)['data'].data
        ).reshape(-1, 2),
    ),
    TableColumn(
        'context_steering/vis/polygon',
        'polygon',
        lambda conf: np.array(
            get_closest_to_timestamp(conf)['data'].data
        ).reshape(-1, 2),
    ),
    TableColumn(
        'context_steering/vis/all_and_pareto_individuals',
        'all_and_pareto_individuals',
        lambda conf: np.array(
            get_closest_to_timestamp(conf)['data'].data
        ).reshape(-1, 5)
    ),
    TableColumn(
        'context_steering/vis/chosen_and_admissible_individuals',
        'chosen_and_admissible_individuals',
        lambda conf: np.array(
            get_latest_in_interval(conf)['data'].data
        ).reshape(-1, 5),
    ),

]
