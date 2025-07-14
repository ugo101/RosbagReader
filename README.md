# Instructions
This Repo should be used for reading data from ros2bags. It provides two different ways of doing this:

1. Read rosbag, convert to csv files which are easy to load into pandas dataframes (useful if dont want to always reload full rosbag in script multiple times)
    - this was a script taken from: https://automaticaddison.com/record-and-play-back-data-using-ros-2-bag-ros-2-jazzy/

2. Read rosbag, save full bag to a dictionary variable with following structure:
    - script written by GPT making use of the rosbag2py package
   

## 1. Convert rosbag to csv file (rosbag_to_csv.py)

1. Before running, make sure the install/setup.bash is sourced for specific project (all messages in the rosbag must be defined for this to work).

2. Import the 'convert_rosbag_to_csv' function and specify bag path and output csv path

3. The fucntion will create a csv file per topic in a pandas dataframe (rows=timestamps, columns = message data fields)

## 2. Rosbag directly to dictionary (rosbag_reader.py)

1. Before running, make sure the install/setup.bash is sourced for specific project (all messages in the rosbag must be defined for this to work).

2. Import the 'read_rosbag_to_dict' function and specify bag path

3. Function returns a data dictionary:
    - data['topic'] = [(timestampt1, msg1), (timestampt2, msg2), etc...] for all topics
    - to access specific message fields, do same as in ROS i.e. for odometry position: data[/odometry/observer][time_index][1].pose.pose.position.x
