# Export Data from ROS
## Workflow
1. Create a ros-bag
   - [Recording](https://docs.ros.org/en/foxy/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html) `ros-bag`s with ros2
   - If necessary override the [Quality of Service](https://docs.ros.org/en/foxy/Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html) policy
2. Convert the ros-bag to a pandas dataframe
   - Use the `data_aggregation.py` script, for more infos run `python3 experiment_measurement/data_aggregation.py --help`
   - If the given configs are not suitable for your project create your own inside `experiment_measurement/config`
3. Analyze the dataframe
