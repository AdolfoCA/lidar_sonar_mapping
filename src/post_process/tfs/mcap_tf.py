"""
This script creates an MCAP rosbag file with a static TF message. 
The script reads a YAML file with the TF configuration and creates a static TF message 
with the given configuration that will be published every second to the "/tf_static" topic.

ARGUMENTS:
- tf_config: The YAML file with the TF configuration.
- output: The output directory to create and write the rosbag file.
- start_time: The start time of the rosbag file in seconds.
- end_time: The end time of the rosbag file in seconds.

USAGE:
python3 mcap_tf.py <tf_config> <output> <start_time> <end_time>

EXAMPLE:
python3 mcap_tf.py /path/to/tf_config.yaml /path/to/output 0 1000000000

NOTE: 
This script uses rosbag2_py with ROS2 Humble. At this point, rosbag2_py does not 
support the SequentialReader to read the rosbag metadata and get the start and end time.
These must therefore be provided as arguments.
"""
import argparse
import os
import rosbag2_py
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
from tf_handler import get_tfs_from_yaml, format_tf_msg

TOPIC_NAME = "/tf_static"


def write_to(tf_path: str, start_time: int, end_time: int, output_path: str) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="/tf_static", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"
        )
    )

    num_iter = int((end_time - start_time) // 1e9)

    tf_config = get_tfs_from_yaml(tf_path)

    for i in range(num_iter):
        ts = start_time + (i * 1e9)
        ros_time = Time(sec=int(ts // 1e9), nanosec=int(ts % 1e9))
        msg = format_tf_msg(ros_time, tf_config)
        writer.write(TOPIC_NAME, serialize_message(msg), int(ts))

    del writer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tf_config", help="YAML file with the TF configuration")
    parser.add_argument("output", help="output directory to create and write to")
    parser.add_argument(
        "start_time", help="Start time of the rosbag file in nanoseconds"
    )
    parser.add_argument("end_time", help="End time of the rosbag file in nanoseconds")

    args = parser.parse_args()

    if not os.path.exists(args.tf_config):
        raise FileNotFoundError(f"File {args.tf_config} not found.")
    if not os.path.isfile(args.tf_config):
        raise IsADirectoryError(f"{args.tf_config} is a directory.")
    if os.path.exists(args.output):
        raise FileExistsError(f"Directory {args.output} already exists.")

    start_time = int(args.start_time) * 1e9
    end_time = int(args.end_time) * 1e9

    write_to(
        tf_path=args.tf_config,
        output_path=args.output,
        start_time=start_time,
        end_time=end_time,
    )


if __name__ == "__main__":
    main()
