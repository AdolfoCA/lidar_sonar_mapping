import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from datetime import datetime
from scipy import signal

def load_optitrack_data(csv_path):
    # Load the CSV optitrack file using pandas
    df = pd.read_csv(csv_path, header=None)

    # Get start timestamp
    opt_start_timestamp = datetime.strptime(df.iloc[0, 11], "%Y-%m-%d %I.%M.%S.%f %p").timestamp()

    df = df.iloc[6:].reset_index(drop=True)
    df = df.iloc[:, : 9+12]  # Add the four balls

    # Get points, timestamps and quaternions representing orientation
    df_timestamps = df.iloc[:, 1]
    df_quaternions = df.iloc[:, 2:6]
    df_points = df.iloc[:, 6:9]
    df_balls = df.iloc[:, 9:21]

    # Convert the dataframe to a numpy array
    opt_points = df_points.to_numpy().astype(float)
    opt_quaternions = df_quaternions.to_numpy().astype(float)
    opt_timestamps = df_timestamps.to_numpy().astype(float)
    opt_balls = df_balls.to_numpy().astype(float)

    before_filter = opt_timestamps.shape[0]
    # Remove points that have a nan value, ie not all balls are tracked
    valid_indices = ~np.isnan(opt_balls).any(axis=1)
    opt_timestamps = opt_timestamps[valid_indices]
    opt_quaternions = opt_quaternions[valid_indices]
    opt_points = opt_points[valid_indices]

    print(f"Removed {before_filter - opt_timestamps.shape[0]} points out of {before_filter}")
    # Convert all the optitrack timestamps to unix time
    opt_timestamps = opt_timestamps + opt_start_timestamp

    # Create Pose messages for each timestamp
    poses = []
    for pos, quat in zip(opt_points, opt_quaternions):
        pose = Pose()
        x, y, z = pos  # Unpacking x, y, z
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qx, qy, qz, qw = quat  # Unpacking qx, qy, qz, qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        poses.append(pose)

    # Create a Pandas DataFrame
    df = pd.DataFrame({'pose': poses}, index=opt_timestamps)
    df.index.name = 'timestamp'  # Naming the index

    return df


def smooth_poses(df, order=12, cutoff=1.0):
    # Extract positions and orientations
    positions = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in df['pose']])
    orientations = np.array([[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] for pose in df['pose']])

    # Smoothing the data using filtfilt
    opt_timestamps = df.index   # Convert timestamps to seconds
    fs = opt_timestamps.shape[0] / (opt_timestamps[-1] - opt_timestamps[0])

    print(fs)

    # Compute filter coefficients
    b, a = signal.butter(order, cutoff / (0.5 * fs), btype='low')

    smoothed_orientations = []
    for col in orientations.T:
        smoothed_orientations.append(signal.filtfilt(b, a, col, padlen=100))

    smoothed_orientations = np.asarray(smoothed_orientations).T

    # Create smoothed Pose messages
    smoothed_poses = []
    for pos, quat in zip(positions, smoothed_orientations):
        pose = Pose()
        x, y, z = pos
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qx, qy, qz, qw = quat
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        smoothed_poses.append(pose)

    # Create a Pandas DataFrame
    smoothed_df = pd.DataFrame({'pose': smoothed_poses}, index=df.index)
    smoothed_df.index.name = 'timestamp'  # Naming the index

    return smoothed_df

# Example usage:
# raw_df = load_optitrack_data('/../data/raw/sonar_0_walkaround.csv')
# smoothed_df = smooth_poses(raw_df)