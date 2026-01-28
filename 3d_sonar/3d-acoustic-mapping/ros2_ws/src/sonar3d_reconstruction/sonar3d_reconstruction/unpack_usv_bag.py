# Author: Jonas Poulsen
# Unpacks a bag file with USV data and saves the data as pandas dataframes
# GitHub Copilot was used in the development of this script

import rosbag2_py
import pandas as pd
import argparse
import os
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_messages(input_bag: str):
    """Read messages from a bag file and yield them one by one."""
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

def unpack_usv_bag(bag_path):
    """Unpack a USV bag file and pack the data as pandas dataframes."""
    data = {}
    for topic, msg, timestamp in read_messages(bag_path):
        if topic not in data:
            data[topic] = []
        data[topic].append([timestamp, msg])
    
    dfs = {}
    for topic, messages in data.items():
        df = pd.DataFrame(messages, columns=['timestamp', 'message'])
        df.set_index('timestamp', inplace=True)
        dfs[topic] = df
    
    return dfs

def main():
    """Main function to unpack a USV bag file and save the data as pandas dataframes.
    
    Usage:
    python unpack_usv_bag.py input.bag output_folder
    """
    parser = argparse.ArgumentParser(description="Unpack a USV bag file and save the data as pandas dataframes")
    parser.add_argument("input", help="input bag path (folder or filepath) to read from")
    parser.add_argument("output", help="output folder to save the dataframes")

    args = parser.parse_args()
    dfs = unpack_usv_bag(args.input)
    
    if not os.path.exists(args.output):
        os.makedirs(args.output)
    
    for topic, df in dfs.items():
        output_path = os.path.join(args.output, f"{topic.replace('/', '_')}.pkl")
        df.to_pickle(output_path)
        print(f"Saved DataFrame for topic: {topic} to {output_path}")

if __name__ == "__main__":
    main()