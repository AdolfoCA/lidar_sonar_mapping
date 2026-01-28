from nmea_msgs.msg import Sentence
from typing import List, Any, Dict
from pynmeagps import NMEAReader, NMEAMessage
import pymap3d as pm


class GPSDataHandler:
    _HEADER = [
        "timestamp",  # in nanoseconds
        "latitude",  # in degrees
        "longitude",  # in degrees
        "altitude",  # in meters
        "latitude_std",  # 1 sigma error, in meters
        "longitude_std",  # 1 sigma error, in meters
        "altitude_std",  # 1 sigma error, in meters
        "x",  # ENU x-coordinate
        "y",  # ENU y-coordinate
        "z",  # ENU z-coordinate
        "orientation",  # "orientation" is the orientation of the GPS receiver
        "hdop",  # Horizontal Dilution of Precision
        "vdop",  # Vertical Dilution of Precision
        "pdop",  # Position Dilution of Precision
        "satellites",  # Number of satellites used in the solution
    ]

    def __init__(self, talker: str = "gps"):
        # GPS data
        self._timestamp = None
        self._latitude = None
        self._longitude = None
        self._altitude = None
        self._latitude_err = None
        self._longitude_err = None
        self._altitude_err = None
        self._orientation = None
        self._x = None
        self._y = None
        self._z = None
        self._hdop = None
        self._vdop = None
        self._pdop = None
        self._satellites = None

        self._lat0 = None
        self._lon0 = None
        self._h0 = None

        self._write_flag = True

    def write(self) -> List[Any]:
        self._write_flag = False
        return [
            self.timestamp,
            self.latitude,
            self.longitude,
            self.altitude,
            self.latitude_error,
            self.longitude_error,
            self.altitude_error,
            self.x,
            self.y,
            self.z,
            self.orientation,
            self.hdop,
            self.vdop,
            self.pdop,
            self.satellites,
        ]

    def update(self, sentence: Sentence) -> None:
        try:
            nmea_msg = NMEAReader.parse(sentence.sentence)
        except Exception as e:
            print(f"Skippong NMEA message: {e}")
            return

        timestamp = sentence.header.stamp.sec * 1e9 + sentence.header.stamp.nanosec
        if (nmea_msg is None) or (nmea_msg.msgID is None):
            return

        match nmea_msg.msgID:
            case "GLL":
                self._update_gll(timestamp, nmea_msg)
            case "GST":
                self._update_gst(timestamp, nmea_msg)
            case "GGA":
                self._update_gga(timestamp, nmea_msg)
            case "GSA":
                self._update_gsa(timestamp, nmea_msg)
            case _:
                return

    def is_ready(self) -> bool:
        return self._is_key_data_available() and self._is_write_ready()

    def _is_key_data_available(self) -> bool:
        return (
            self.timestamp is not None
            and self.latitude is not None
            and self.longitude is not None
        )

    def _is_write_ready(self) -> bool:
        return self._write_flag

    def _update_gll(self, timestamp: int, msg: NMEAMessage) -> None:
        if msg.msgID != "GLL":
            return

        self.timestamp = timestamp
        self.latitude = msg.lat
        self.longitude = msg.lon

        if self.lat0 is None or self.lon0 is None or self.h0 is None:
            self.lat0 = msg.lat
            self.lon0 = msg.lon
            self.h0 = 0

        x, y, z = pm.geodetic2enu(msg.lat, msg.lon, 0, self.lat0, self.lon0, 0)

        self.x = x
        self.y = y
        self.z = z

    def _update_gst(self, timestamp: int, msg: NMEAMessage) -> None:
        if msg.msgID != "GST":
            return

        self.timestamp = timestamp

        self.latitude_error = msg.stdLat
        self.longitude_error = msg.stdLong
        self.altitude_error = msg.stdAlt
        self.orientation = msg.orient

    def _update_gga(self, timestamp: int, msg: NMEAMessage) -> None:
        if msg.msgID != "GGA":
            return

        self.timestamp = timestamp
        self.altitude = msg.alt
        self.hdop = msg.HDOP

    def _update_gsa(self, timestamp: int, msg: NMEAMessage) -> None:
        if msg.msgID != "GSA":
            return

        self.timestamp = timestamp
        self.satellites = len(
            [k for k, v in self._get_satellites(msg).items() if v is not ""]
        )
        self.pdop = msg.PDOP
        self.vdop = msg.VDOP

    def _get_satellites(self, msg: NMEAMessage) -> Dict[str, Any]:
        keywords = [f"svid_{i:02d}" for i in range(1, 13)]
        return {k: getattr(msg, k) for k in keywords}

    @property
    def timestamp(self):
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        self._timestamp = value

    @property
    def latitude(self):
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        self._latitude = value
        self._write_flag = True

    @property
    def longitude(self):
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        self._longitude = value
        self._write_flag = True

    @property
    def altitude(self):
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        self._altitude = value
        self._write_flag = True

    @property
    def latitude_error(self):
        return self._latitude_err

    @latitude_error.setter
    def latitude_error(self, value):
        self._latitude_err = value

    @property
    def longitude_error(self):
        return self._longitude_err

    @longitude_error.setter
    def longitude_error(self, value):
        self._longitude_err = value

    @property
    def altitude_error(self):
        return self._altitude_err

    @altitude_error.setter
    def altitude_error(self, value):
        self._altitude_err = value

    @property
    def hdop(self):
        return self._hdop

    @hdop.setter
    def hdop(self, value):
        self._hdop = value

    @property
    def vdop(self):
        return self._vdop

    @property
    def pdop(self):
        return self._pdop

    @pdop.setter
    def pdop(self, value):
        self._pdop = value

    @property
    def vdop(self):
        return self._vdop

    @vdop.setter
    def vdop(self, value):
        self._vdop = value

    @property
    def satellites(self):
        return self._satellites

    @satellites.setter
    def satellites(self, value):
        self._satellites = value

    @property
    def header(self):
        return self._HEADER

    @property
    def lat0(self):
        return self._lat0

    @lat0.setter
    def lat0(self, value):
        self._lat0 = value
        self._write_flag = True

    @property
    def lon0(self):
        return self._lon0

    @lon0.setter
    def lon0(self, value):
        self._lon0 = value
        self._write_flag = True

    @property
    def h0(self):
        return self._h0

    @h0.setter
    def h0(self, value):
        self._h0 = value
        self._write_flag = True

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value
        self._write_flag = True

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value
        self._write_flag = True

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        self._z = value
        self._write_flag = True

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value
