"""Aggregate the data to one table."""
# from experiment_measurement import data_aggregation_helper
from experiment_measurement import data_aggregation_helper

"""
'table_column_config' specifies the mapping from the rosbag data to a new pandas dataframe.

Each entry in the array defines a column inside the new dataframe. The entry includes the
topic from which the data should be taken, a name for the new column and a function. The function
is called once for every new entry in the new table-column. The input is a TableConfig object which
stores all data of {the current topic and the current robot} in a list, as well as a timestamp and
other data needed to resolve the entry.

If the topic is not associated with a name-space (robot), the topic name must begin with '/'. In
this case the TableConfig object contains all data of the current topic.


table_column_config  ::= table_column, [ table_column, ... ]
table_column         ::= ( topic, new_table_column_name, func )

topic                ::= string
table_column_name    ::= string
func                 ::= TableConfig -> Object
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
        '$\\theta$',
        lambda conf: data_aggregation_helper.quaternion_to_euler(
            data_aggregation_helper.get_latest_in_interval(
                data_aggregation_helper.filter_tf_child_frame_id(conf)
            )['data'].transforms[0].transform.rotation
        )[2],
    ),
    data_aggregation_helper.TableColumn(
        '/clock',
        'sim_time',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].clock.sec,
    ),
#    data_aggregation_helper.TableColumn(
#        'scan',
#        'min_scan',
#        lambda conf: min(
#            data_aggregation_helper.get_latest_in_interval(conf)['data'].ranges
#        ),
#    ),
#    data_aggregation_helper.TableColumn(
#        'nav/cell',
#        'cell',
#        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data,
#    ),
    data_aggregation_helper.TableColumn(
        'nav/plan',
        'plan',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data,
    ),
#    data_aggregation_helper.TableColumn(
#        'nav/opinion',
#        'opinion',
#        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data,
#    ),
#    data_aggregation_helper.TableColumn(
#        'nav/cdm',
#        'cdm',
#        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data,
#    ),
    data_aggregation_helper.TableColumn(
        'nav/goal_completed',
        'goal_count',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data,
    ),
#    data_aggregation_helper.TableColumn(
#        'nav/belief',
#        'belief',
#        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data,
#    ),
    data_aggregation_helper.TableColumn(
        '/command',
        'command',
        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
    ),
#    data_aggregation_helper.TableColumn(
#        'status',
#        'status',
#        lambda conf: data_aggregation_helper.get_latest_in_interval(conf)['data'].data
#    ),
]
