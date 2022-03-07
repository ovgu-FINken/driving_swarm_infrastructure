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
            data_aggregation_helper.get_latest_in_interval(  #
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
    ),
    data_aggregation_helper.TableColumn(
        'scan',
        'obstacle_dist',
        lambda conf: np.array(
            data_aggregation_helper.get_latest_in_interval(conf)['data'].ranges
        ),
    ),
    data_aggregation_helper.TableColumn(
        'cmd_vel',
        'rot_vel',
        lambda conf: data_aggregation_helper.get_earliest_in_next_intervall(conf)[  #
            'data'
        ].angular.z,
    ),
    data_aggregation_helper.TableColumn(
        'cmd_vel',
        'trans_vel',
        lambda conf: data_aggregation_helper.get_earliest_in_next_intervall(conf)[
            'data'
        ].linear.x,
    ),
    data_aggregation_helper.TableColumn(
        'context_steering/vis/params',
        'params',
        lambda conf: data_aggregation_helper.get_all_params_as_tuple_list(conf)
    ),
    data_aggregation_helper.TableColumn(
        'context_steering/samples',
        'samples',
        lambda conf: np.array(
            data_aggregation_helper.get_latest_in_interval(conf)['data'].data
        ).reshape(-1, 2),
    ),
    data_aggregation_helper.TableColumn(
        'context_steering/vis/polygon',
        'polygon',
        lambda conf: np.array(
            data_aggregation_helper.get_closest_to_timestamp(conf)['data'].data
        ).reshape(-1, 2),
    ),
    data_aggregation_helper.TableColumn(
        'context_steering/vis/all_and_pareto_individuals',
        'all_and_pareto_individuals',
        lambda conf: np.array(
            data_aggregation_helper.get_closest_to_timestamp(conf)['data'].data
        ).reshape(-1, 5)
    ),
    data_aggregation_helper.TableColumn(
        'context_steering/vis/chosen_and_admissible_individuals',
        'chosen_and_admissible_individuals',
        lambda conf: np.array(
            data_aggregation_helper.get_latest_in_interval(conf)['data'].data
        ).reshape(-1, 5),
    ),

]
