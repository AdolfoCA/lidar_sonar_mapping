"""
This script generates a timeline plot of the ROS topics in an MCAP file. Non-colored (white)
bars represent the time intervals where no messages were published on the topic.

ARGUMENTS:
- rosbag: The input MCAP file to generate the timeline plot from.

USAGE:
python3 mcap_timeline.py <rosbag.mcap>
"""

import argparse
import os
import logging
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

import rosbag2_py
from datetime import datetime

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MCAPTimeline:
    NS_TO_S = 1e-9
    TIME_THRESHOLD = 1 * 1e9  # seconds
    EXCLUDE_TOPICS = [
        "/rosout",
        "/time_reference",
    ]
    TIMESTAMP_FORMAT = "%Y-%m-%d %H:%M:%S"

    DTU_RED = "#990000"
    DTU_BLUE = "#2F3EEA"
    DTU_GREEN = "#1FD082"

    def __init__(self):
        self.start_datetime = 0
        self.end_datetime = 0
        self.data = {}

    def _read_mcap_contents(self, input_bag: str):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        while reader.has_next():
            topic, _, timestamp = reader.read_next()
            yield topic, timestamp

        del reader

    def _acquire_data(self, input_bag: str) -> None:
        self.data = {}
        for topic, timestamp in self._read_mcap_contents(input_bag):
            self._handle_topic_state(topic, timestamp)

        for topic, data in self.data.items():
            end = data[-1][1]
            if end == -1:
                st = data[-1][0]
                self.data[topic][-1] = (st, self.end_datetime)

    def _handle_topic_state(self, topic: str, timestamp: int) -> None:
        if topic in self.EXCLUDE_TOPICS:
            return

        if self.start_datetime == 0:
            self.start_datetime = timestamp

        self.end_datetime = timestamp

        # if the topic is not in the data, add it
        if topic not in self.data:
            self.data[topic] = []

        # if the topic is empty, add the first timestamp
        if self.data[topic] == []:
            self.data[topic].append((timestamp, -1))
            return

        # if the topic is not empty, check the last tuple
        for key, data in self.data.items():
            last_tuple = data[-1]

            if last_tuple[1] == -1:
                if timestamp - last_tuple[0] > self.TIME_THRESHOLD:
                    self.data[key][-1] = (last_tuple[0], timestamp)
            elif key == topic:
                self.data[key].append((timestamp, -1))

    def plot(self, input_bag: str) -> None:
        self._acquire_data(input_bag)

        def truncate_label(label: str) -> str:
            MAX_LEN = 15
            return label if len(label) <= MAX_LEN else f"...{label[-MAX_LEN:]}"

        # Prepare data for broken bar plot
        y_ticks = []
        y_labels = []
        bars = []
        for i, (topic, data) in enumerate(self.data.items()):
            y_ticks.append(i)

            y_labels.append(truncate_label(topic))
            y_bars = []
            for start, end in data:
                y_bars.append(
                    (
                        datetime.fromtimestamp(start * self.NS_TO_S),
                        datetime.fromtimestamp(end * self.NS_TO_S)
                        - datetime.fromtimestamp(start * self.NS_TO_S),
                    )
                )
            bars.append(y_bars)

        # Create broken bar plot
        _, ax = plt.subplots(figsize=(8, 5))
        for i, y_tick in enumerate(y_ticks):
            ax.broken_barh(
                bars[i],
                (y_tick - 0.4, 0.8),
                facecolors=self.DTU_BLUE if i % 2 == 0 else self.DTU_RED,
            )

        ax.set_ylim(-1, len(self.data))
        ax.set_xlim(
            datetime.fromtimestamp(self.start_datetime * self.NS_TO_S),
            datetime.fromtimestamp(self.end_datetime * self.NS_TO_S),
        )
        ax.set_yticks(y_ticks)
        ax.set_yticklabels(y_labels)

        plt.xticks(rotation=-45)
        ax.set_xlabel("Datetime of ROS Messages")
        ax.set_ylabel("ROS Topics")

        ax.xaxis.grid(True)
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        plt.tight_layout()
        plt.savefig(f"{input_bag}_timeline.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "rosbag", help="input bag path (folder or filepath) to generate plot from"
    )
    args = parser.parse_args()

    if not os.path.exists(args.rosbag) or not os.path.isfile(args.rosbag):
        raise FileNotFoundError(f"File {args.rosbag} does not exist or is invalid.")

    if not args.rosbag.endswith(".mcap"):
        raise ValueError(f"File {args.rosbag} is not an MCAP file.")

    logger.info(f"Generating MCAP timeline for {args.rosbag}...")
    mcap_timeline = MCAPTimeline()
    mcap_timeline.plot(args.rosbag)
    logger.info(f'MCAP timeline generated as "{args.rosbag}_timeline.png"')


if __name__ == "__main__":
    main()
