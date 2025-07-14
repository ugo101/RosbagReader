# Written by GPT

# returns single data dictionary with topics as keys. each topic is then a list of tuples [(timestamp1, msg1), (timestampt2, msg2), etc]
# message data fields can be accessed same as in ros2 e.g. msg.pose.pose.position.x

# NOTE: make sure to change storage_id to 'mcap' if using mcap files, or 'sqlite3' for db3 files

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_rosbag_to_dict(bag_path):
    """
    Reads a ROS 2 bag (.mcap or .db3) into a dictionary of topic -> list of (timestamp, ROS message).
    Keeps native ROS message structures.
    
    :param bag_path: Path to the ROS 2 bag directory.
    :return: Dictionary mapping topic names to lists of (timestamp, message) tuples.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    data = {}

    while reader.has_next():
        topic, serialized_msg, t = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(serialized_msg, msg_type)
        if topic not in data:
            data[topic] = []
        data[topic].append((t, msg))

    return data

if __name__ == "__main__":
    bag_path = '/home/damenadmin/Projects/PoolTesting/Rosbags/DP_test_1_disturbance'
    data = read_rosbag_to_dict(bag_path)
    print(data['/odometry/observer'][0][1].pose.pose.position.x)  # Example access to a specific field