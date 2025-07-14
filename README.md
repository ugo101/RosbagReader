The script to convert rosbags to csv files was taken from https://automaticaddison.com/record-and-play-back-data-using-ros-2-bag-ros-2-jazzy/

# Instructions

## 1. Convert rosbag to csv file

1. Before running, make sure the install/setup.bash is sourced for specific project (all messages in the rosbag must be defined in install/setup.bash for this to work).

2. Specify Input rosbag path (can also be folder of multiple rosbags) and output csv folder where to save csv files at bottom of rosbag_to_csv.py script.

3. Run script, for each bag, a csv folder shall be generated per topic (all bundled in a folder with rosbag name) where each columns corresponds to a seperate data entry in the topic.

NOTE: additional plotting function is specific to plotting observer data for DAVe in pool!!

