## NOTE: BELOW SCRIPT TAKEN FROM: https://automaticaddison.com/record-and-play-back-data-using-ros-2-bag-ros-2-jazzy/
#!/usr/bin/env python3
"""
Convert ROS 2 bag files to CSV format with automatic field extraction.

This version supports *batch conversion* of all bag folders inside a parent folder,
saving their CSVs in a corresponding output root.

Usage (no command-line args needed here):
  python3 ros2bag_to_csv.py

Author: Adapted for batch automation
"""

import csv
import os

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from std_msgs.msg import String


def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp

    del reader


def flatten_ros_message(msg, prefix=''):
    flat = {}
    if hasattr(msg, '__slots__'):
        for field_name in msg.__slots__:
            value = getattr(msg, field_name)
            key = f"{prefix}{field_name}"
            if hasattr(value, '__slots__'):
                # Nested ROS message
                flat.update(flatten_ros_message(value, prefix=key + '_'))
            elif isinstance(value, (list, tuple)):
                # Store arrays as comma-separated strings
                flat[key] = ','.join(str(v) for v in value)
            else:
                flat[key] = value
    else:
        flat[prefix.rstrip('_')] = msg
    return flat



def convert_rosbag_to_csv(input_bag, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    topic_files = {}
    topic_writers = {}
    topic_headers = {}

    for topic, msg, timestamp in read_messages(input_bag):
        if isinstance(msg, String):
            flat_data = {'data': msg.data}
        else:
            flat_data = flatten_ros_message(msg)

        flat_data = {'timestamp': timestamp, **flat_data}

        if topic not in topic_files:
            output_file = os.path.join(output_folder, topic.replace('/', '_') + '.csv')
            topic_files[topic] = open(output_file, mode='w', newline='', encoding='utf-8')
            topic_writers[topic] = csv.writer(topic_files[topic])
            topic_headers[topic] = list(flat_data.keys())
            topic_writers[topic].writerow(topic_headers[topic])

        current_fields = list(flat_data.keys())
        if set(current_fields) != set(topic_headers[topic]):
            topic_writers[topic].writerow(current_fields)
            topic_headers[topic] = current_fields

        row = [flat_data.get(col, '') for col in topic_headers[topic]]
        topic_writers[topic].writerow(row)

    for file in topic_files.values():
        file.close()


if __name__ == "__main__":
    # Batch conversion logic
    ROSBAG_FOLDER = '/home/damenadmin/Projects/PoolTesting/Rosbags'
    CSV_OUTPUT_ROOT = '/home/damenadmin/Projects/PoolTesting/Rosbags_csv'

    # Get all subdirectories in ROSBAG_FOLDER
    list_bag_paths = [
        os.path.join(ROSBAG_FOLDER, name)
        for name in os.listdir(ROSBAG_FOLDER)
        if os.path.isdir(os.path.join(ROSBAG_FOLDER, name))
    ]

    # Process each bag folder
    for bag_path in list_bag_paths:
        bag_name = os.path.basename(bag_path)
        output_folder = os.path.join(CSV_OUTPUT_ROOT, bag_name)
        print(f"Converting bag: {bag_path} -> {output_folder}")
        convert_rosbag_to_csv(bag_path, output_folder)

    print("All bags converted successfully.")
