import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from tqdm import tqdm
import csv


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
        if topic != "/ouster/imu":
            continue
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp

    print("Reading messages from bag... done")
    del reader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    args = parser.parse_args()

    CSV_FILENAME = args.input + ".csv"
    csv_file = open(CSV_FILENAME, "w")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Norm"])
    for _, msg, timestamp in read_messages(args.input):
        if isinstance(msg, Imu):
            norm = (
                msg.linear_acceleration.x**2
                + msg.linear_acceleration.y**2
                + msg.linear_acceleration.z**2
            ) ** 0.5
            csv_writer.writerow([timestamp, norm])

    csv_file.close()


if __name__ == "__main__":
    main()
