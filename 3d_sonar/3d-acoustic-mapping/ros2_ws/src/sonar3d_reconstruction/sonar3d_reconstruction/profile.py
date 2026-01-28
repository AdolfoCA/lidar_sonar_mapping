# Credits to Aimas Lund for the original implementation of this class

from dataclasses import dataclass
from enum import Enum
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt


@dataclass
class Profile:
    """
    The Profile class is a dataclass that contains the data of a single profile, i.e. points in 3D space
    for a single ping.
    Attributes:
        range (np.ndarray): The range of the points in the profile.
        bearing (np.ndarray): The bearing of the points in the profile.
        x (np.ndarray): The x-coordinate of the points in the profile.
        y (np.ndarray): The y-coordinate of the points in the profile.
        z (np.ndarray): The z-coordinate of the points in the profile.
        delay (np.ndarray): The delay of the points in the profile.
        intensity (np.ndarray): The intensity of the points in the profile.
        valid (np.ndarray): A boolean array indicating whether the points are valid or not.

    Methods:
        filter_valid(): Returns a new Profile object containing only the valid points.
        This uses the valid attribute as a mask to filter the other attributes.
    """

    range: np.ndarray
    bearing: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    delay: np.ndarray
    intensity: np.ndarray
    valid: np.ndarray

    def __len__(self):
        return len(self.range)

    def filter_valid(self):
        return Profile(
            self.range[self.valid],
            self.bearing[self.valid],
            self.x[self.valid],
            self.y[self.valid],
            self.z[self.valid],
            self.delay[self.valid],
            self.intensity[self.valid],
            self.valid[self.valid],
        )
    
    def combine(self, profile: 'Profile') -> 'Profile':
        return Profile(
            np.concatenate((self.range, profile.range), axis=0),
            np.concatenate((self.bearing, profile.bearing), axis=0),
            np.concatenate((self.x, profile.x), axis=0),
            np.concatenate((self.y, profile.y), axis=0),
            np.concatenate((self.z, profile.z), axis=0),
            np.concatenate((self.delay, profile.delay), axis=0),
            np.concatenate((self.intensity, profile.intensity), axis=0),
            np.concatenate((self.valid, profile.valid), axis=0),
        )
    
    def sort(self, by: str = "y") -> 'Profile':
        if by == "range":
            idx = np.argsort(self.range)
        elif by == "y":
            idx = np.argsort(self.y)
        elif by == "x":
            idx = np.argsort(self.x)
        elif by == "z":
            idx = np.argsort(self.z)
        else:
            raise ValueError(f"Invalid sorting key: {by}")
        
        
        return Profile(
            self.range[idx],
            self.bearing[idx],
            self.x[idx],
            self.y[idx],
            self.z[idx],
            self.delay[idx],
            self.intensity[idx],
            self.valid[idx],
        )
    
    def plot(self):
        """Plot the profile on a given axis."""
        # 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        cmap = cm.get_cmap("tab20", len(self.intensity))
        ax.scatter(self.x, self.y, self.z, c=self.intensity, cmap=cmap)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        return ax

class ProfileType(Enum):
    maximum = "maximum"
    nearest = "nearest"
    falling = "falling"
    nearest_falling = "nearest_falling"
    patches = "patches"
