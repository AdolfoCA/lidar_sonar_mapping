import os
import pandas as pd

def load_and_combine_sonar_data(sonar_data_path):
    """
    Load and combine sonar data from the specified path.

    Parameters:
    sonar_data_path (str): The path to the directory containing the sonar data.

    Returns:
    DataFrame: A combined DataFrame with resampled sonar data.
    """
    sonar_oculus_path = os.path.join(sonar_data_path, "_oculus_sonar_image.pkl")
    sonar_blueview_path = os.path.join(sonar_data_path, "_blueview_sonar_image.pkl")
    transforms_path = os.path.join(sonar_data_path, "_tf_static.pkl")

    # Read pickled DataFrames
    sonar_oculus_df = pd.read_pickle(sonar_oculus_path)
    sonar_blueview_df = pd.read_pickle(sonar_blueview_path)
    transforms_df = pd.read_pickle(transforms_path)

    # Combine sonar data
    sonar_df = combine_sonar_data(sonar_oculus_df, sonar_blueview_df, transforms_df)

    return sonar_df

def combine_sonar_data(sonar_oculus_df, sonar_blueview_df, transforms_df):
    """
    Combine sonar data from the Oculus and BlueView sonar sensors.

    Parameters:
    sonar_oculus_df (DataFrame): DataFrame containing Oculus sonar data.
    sonar_blueview_df (DataFrame): DataFrame containing BlueView sonar data.
    transforms_df (DataFrame): DataFrame containing transform data.

    Returns:
    DataFrame: A combined DataFrame with resampled sonar data.
    """
    # Combine sonar data
    # Combine and resample the sonar data. Blueview runs at 10 Hz and Oculus at 15 Hz
    sonar_blueview_df = sonar_blueview_df.rename(columns={"message": "blueview_image"})
    sonar_oculus_df = sonar_oculus_df.rename(columns={"message": "oculus_image"})
    transforms_df = transforms_df.rename(columns={"message": "tf_static"})
    sonar_df = pd.concat([sonar_oculus_df, sonar_blueview_df, transforms_df], axis=1)
    sonar_df.index = pd.to_datetime(sonar_df.index)
    sonar_df = sonar_df.resample("100ms").last()

    # For missing transforms, interpolate the previous transform
    sonar_df["tf_static"] = sonar_df["tf_static"].interpolate(method="pad")

    # Drop rows with NaN values
    sonar_df = sonar_df.dropna()

    return sonar_df