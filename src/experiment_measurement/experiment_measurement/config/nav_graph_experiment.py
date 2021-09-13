"""Aggregate the data to one table."""
from experiment_measurement import data_aggregation_helper
#import data_aggregation_helper

"""
table_column_config  ::= table_column, [ df_aggregated_topics ]
table_column         ::= ( topic, table_column_name, func )

topic                ::= string
table_column_name    ::= string
func                 ::= TableConfig -> Object

message_dataframe    ::= pd.Series      # All messages to one topic and robot
time                 ::= number
"""
table_column_config = [
    data_aggregation_helper.TableColumn(
        '/plan',
        'plan',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data,
    ),
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
        '/tf',
        'x',
        lambda conf: data_aggregation_helper.get_latest_in_interval(
            data_aggregation_helper.filter_tf_child_frame_id(conf)
        )[
            'data'
        ].transforms[0].transform.translation.x,
    ),
    data_aggregation_helper.TableColumn(
        '/tf',
        'y',
        lambda conf: data_aggregation_helper.get_latest_in_interval(
            data_aggregation_helper.filter_tf_child_frame_id(conf)
        )[
            'data'
        ].transforms[0].transform.translation.y,
    ),
    data_aggregation_helper.TableColumn(
        '/tf',
        'theta',
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
    ),
    data_aggregation_helper.TableColumn(
        'scan',
        'min_obstacle_dist',
        lambda conf: min(
            data_aggregation_helper.get_latest_in_interval(conf)['data'].ranges
        ),
    ),
    data_aggregation_helper.TableColumn(
        'nav/desired',
        'desired_x',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].pose.position.x,
    ),
    data_aggregation_helper.TableColumn(
        'nav/desired',
        'desired_y',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].pose.position.y,
    ),
    data_aggregation_helper.TableColumn(
        'nav/desired',
        'desired_theta',
        lambda conf: data_aggregation_helper.quaternion_to_euler(
            data_aggregation_helper.get_latest_in_interval(conf)['data'].pose.orientation
        )[2],
    ),
    data_aggregation_helper.TableColumn(
        '/command',
        'command',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    ),
    data_aggregation_helper.TableColumn(
        'status',
        'status',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    ),
    data_aggregation_helper.TableColumn(
        'cell',
        'cell',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    ),
    data_aggregation_helper.TableColumn(
        'nav/plan',
        'current_plan',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    )
]
