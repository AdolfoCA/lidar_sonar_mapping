import folium
from colour import Color
import pandas as pd
import os
import argparse

POINT_SUPSAMPLE = 7
DTU_RED = "#990000"
DTU_BLUE = "#2F3EEA"
DATETIME_FORMAT = "%Y-%m-%d %H:%M:%S"


def _read_csv(csv_file: str) -> pd.DataFrame:
    df = pd.read_csv(csv_file, delimiter=",", header=0)
    df["Timestamp"] = pd.to_datetime(df["Timestamp"])
    return df


def _find_avg_lat_lon(df: pd.DataFrame) -> tuple:
    avg_lat = df["Lat"].mean()
    avg_lon = df["Lon"].mean()
    return avg_lat, avg_lon


def generate_map(df: pd.DataFrame, output_file: str) -> None:
    m = folium.Map(location=list(_find_avg_lat_lon(df)), zoom_start=14)

    start_color = Color(DTU_RED)
    end_color = Color(DTU_BLUE)

    start_datetime = df["Timestamp"].iloc[0]
    end_datetime = df["Timestamp"].iloc[-1]

    folium.Marker(
        location=[df["Lat"].iloc[0], df["Lon"].iloc[0]],
        popup=f"Start Time: {start_datetime.strftime(DATETIME_FORMAT)}",
        icon=folium.Icon(icon="play"),
        color=DTU_RED,
    ).add_to(m)
    folium.Marker(
        location=[df["Lat"].iloc[-1], df["Lon"].iloc[-1]],
        popup=f"End Time: {end_datetime.strftime(DATETIME_FORMAT)}",
        icon=folium.Icon(icon="flag"),
        color=DTU_BLUE,
    ).add_to(m)

    colors = list(start_color.range_to(end_color, len(df) - 1))
    for i in range(len(df) - 1):
        folium.PolyLine(
            locations=[
                [df["Lat"].iloc[i], df["Lon"].iloc[i]],
                [df["Lat"].iloc[i + 1], df["Lon"].iloc[i + 1]],
            ],
            color=colors[i].hex,
            weight=3.5,
            opacity=1,
            fill=True,
        ).add_to(m)

    m.save(output_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="input GPS CSV file to read from")
    parser.add_argument("output", help="output HTML file to write to")
    args = parser.parse_args()

    if os.path.exists(args.output):
        raise FileExistsError(f"File {args.output} already exists.")
    if not os.path.exists(args.input):
        raise FileNotFoundError(f"File {args.input} not found.")

    df = _read_csv(args.input)
    generate_map(df, args.output)


