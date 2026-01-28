import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nmea_msgs.msg import Sentence
from pynmeagps import NMEAReader
import csv
import os
from datetime import datetime
from gps_handler import GPSDataHandler


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
        if topic != "/nmea_sentence":
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

    if not os.path.isdir("./csvs"):
        os.mkdir("./csvs")

    gps_handler = GPSDataHandler()
    CSV_FILENAME = f"./csvs/gps_data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    csv_file = open(CSV_FILENAME, "w")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(GPSDataHandler._HEADER)

    for _, ros_msg, _ in read_messages(args.input):
        if not isinstance(ros_msg, Sentence):
            continue

        gps_handler.update(ros_msg)

        if gps_handler.is_ready():
            csv_writer.writerow(gps_handler.write())

    csv_file.close()


if __name__ == "__main__":
    main()
