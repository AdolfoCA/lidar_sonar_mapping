import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Path
import csv
import os
import argparse
from datetime import datetime

_TOPIC_NAME = "/lio_sam/mapping/path"

def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()
    print("Topics in bag:")
    for topic_type in topic_types:
        print(f"  {topic_type.name} ({topic_type.type})")

    print("Reading messages from bag...")

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != _TOPIC_NAME:
            continue

        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp

    print("Reading messages from bag... done")
    del reader

def extract_paths(input_bag: str) -> Path:
    path_data = None

    for _, ros_msg, _ in read_messages(input_bag):
        if not isinstance(ros_msg, Path):
            continue

        path_data = ros_msg

    return path_data

def write_csv(csv_writer, path_data: Path):
    for pose_stamped in path_data.poses:
        csv_writer.writerow([
            pose_stamped.header.stamp.sec * 1e9 + pose_stamped.header.stamp.nanosec,
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
        ])

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    args = parser.parse_args()

    if not os.path.isdir("./csvs"):
        os.mkdir("./csvs")

    CSV_FILENAME = f"./csvs/path_data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    csv_file = open(CSV_FILENAME, "w")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["timestamp", "x", "y", "z", "ori_x", "ori_y", "ori_z", "ori_w"])

    path_data = extract_paths(args.input)

    if path_data is not None:
        write_csv(csv_writer, path_data)

    csv_file.close()


if __name__ == "__main__":
    main()
