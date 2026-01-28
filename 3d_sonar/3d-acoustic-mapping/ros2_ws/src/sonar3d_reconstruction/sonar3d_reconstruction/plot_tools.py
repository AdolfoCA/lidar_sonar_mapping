import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from sonar3d_reconstruction.edge_detection import _get_leading_edges
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_sonar_image(ax, sonar_image, ranges, beams, title, title_font_size=10, cmap="inferno"):
    ax.imshow(sonar_image, cmap=cmap, vmin=0, vmax=255)
    ax.set_title(title, fontsize=title_font_size)
    ax.invert_yaxis()
    range_ticks = np.linspace(0, sonar_image.shape[0] - 1, num=10, dtype=int)
    bearing_ticks = np.linspace(0, sonar_image.shape[1] - 1, num=5, dtype=int)
    range_labels = [f"{ranges[tick]:.1f}" for tick in range_ticks]
    bearing_labels = [
        f"{np.round(np.rad2deg(beams[tick]), 0):.0f}" for tick in bearing_ticks
    ]
    ax.set_xticks(bearing_ticks)
    ax.set_xticklabels(bearing_labels)
    ax.set_xlabel("Bearing (deg.)")
    ax.set_yticks(range_ticks)
    ax.set_yticklabels(range_labels)
    ax.set_ylabel("Range (m)")
    ax.grid(False)


def plot_sonar_image_cartesian(ax, sonar_image, y, x, title=None, title_font_size=10, cmap="inferno"):
    ax.scatter(y, x, c=sonar_image.flatten(), cmap=cmap, vmin=0, vmax=255, s=1)
    if title is not None:
        ax.set_title(title, fontsize=title_font_size)
    ax.set_xlabel("y [m]")
    ax.set_ylabel("x [m]")
    ax.axis("equal")


def plot_tf_message(global_transforms, fig_size=(3, 5), plane="XZ"):
    """
    Plots the 2D transforms on the X-Z plane from a TFMessage, including rotation.

    Args:
        tf_message (tf2_msgs.msg.TFMessage): The TFMessage containing transforms to plot.
        fig_size (tuple, optional): The size of the figure. Defaults to (3, 5).
        plane (str, optional): The plane to plot the transforms on. Defaults to "XZ". XYZ yields a 3D plot.
    """
    fig, ax = plt.subplots(figsize=fig_size)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("TF Transforms on X-Z Plane")

    # Plot transforms
    for frame, (translation, rotation) in global_transforms.items():
        x, y, z = translation[0], translation[1], translation[2]

        # Define orientation axes
        axes_length = 0.1
        x_axis = rotation.apply([axes_length, 0, 0])
        z_axis = rotation.apply([0, 0, axes_length])

        # Plot position
        ax.scatter(x, z, label=frame)
        ax.text(x + 0.03, z + 0.03, frame)

        # Plot orientation
        ax.quiver(x, z, x_axis[0], x_axis[2], color="r", label="x-axis")
        ax.quiver(x, z, z_axis[0], z_axis[2], color="b", label="z-axis")

    custom_lines = [
        plt.Line2D([0], [0], color="r", lw=2, label="x-axis"),
        plt.Line2D([0], [0], color="b", lw=2, label="z-axis"),
    ]

    ax.legend(
        handles=custom_lines,
        loc="lower left",
        bbox_to_anchor=(-0.3, -0.25),
        frameon=False,
    )
    ax.axis("equal")
    ax.grid(True)

    return fig


# Function "set_size" IS FROM https://jwalton.info/Embed-Publication-Matplotlib-Latex/
def set_size(width, fraction=1, subplots=(1, 1), height_ratio=1):
    """Set figure dimensions to avoid scaling in LaTeX.

    Parameters
    ----------
    width: float or string
            Document width in points, or string of predined document type
    fraction: float, optional
            Fraction of the width which you wish the figure to occupy
    subplots: array-like, optional
            The number of rows and columns of subplots.
    Returns
    -------
    fig_dim: tuple
            Dimensions of figure in inches
    """
    if width == "thesis":
        width_pt = 426.79135
    elif width == "beamer":
        width_pt = 307.28987
    elif width == "project":
        width_pt = 454.10574
    else:
        width_pt = width

    # Width of figure (in pts)
    fig_width_pt = width_pt * fraction
    # Convert from pt to inches
    inches_per_pt = 1 / 72.27

    # Golden ratio to set aesthetic figure height
    # https://disq.us/p/2940ij3
    golden_ratio = (5**0.5 - 1) / 2

    # Figure width in inches
    fig_width_in = fig_width_pt * inches_per_pt
    # Figure height in inches
    fig_height_in = fig_width_in * golden_ratio * (subplots[0] / subplots[1])
    fig_height_in = fig_height_in * height_ratio

    return (fig_width_in, fig_height_in)


def set_style():
    sns.set_context("paper")
    sns.set(
        context="paper",
        style="whitegrid",
        font="serif",
        font_scale=1,
        rc={
            "xtick.bottom": True,
            "ytick.left": True,
            "axes.edgecolor": ".15",
            "lines.solid_capstyle": "butt",
            "figure.dpi": 600,
            "savefig.dpi": 600,
            "axes.labelsize": 10,
            "font.size": 10,
            # Make the legend/label fonts a little smaller
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
        },
    )


def get_dtu_color_palette(color_name=None, format="rgb"):
    """
    NOTE: THIS FUNCTION IS GENERATED USING CHATGPT!

    Returns the specified format (RGB, CMYK, or HEX) for a given color or the entire color palette.

    Args:
        color_name (str, optional): The name of the color to retrieve. Default is None.
        format (str, optional): The format of the color values ('rgb', 'cmyk', or 'hex'). Default is 'rgb'.

    Returns:
        if None: dict: The entire color palette.
        else: The values of the specified color in the specified format.
    """

    def rgb_to_hex(rgb):
        return "#" + "".join(f"{int(c * 255):02x}" for c in rgb)

    palette = {
        "dtured": ((0.6, 0, 0), (0, 0.91, 0.72, 0.23)),
        "blue": ((0.1843, 0.2431, 0.9176), (0.88, 0.76, 0, 0)),
        "brightgreen": ((0.1216, 0.8157, 0.5098), (0.69, 0, 0.66, 0)),
        "navyblue": ((0.0118, 0.0588, 0.3098), (1, 0.9, 0, 0.6)),
        "yellow": ((0.9647, 0.8157, 0.3019), (0.05, 0.17, 0.82, 0)),
        "orange": ((0.9882, 0.4627, 0.2039), (0, 0.65, 0.86, 0)),
        "pink": ((0.9686, 0.7333, 0.6941), (0, 0.35, 0.26, 0)),
        "grey": ((0.8549, 0.8549, 0.8549), (0, 0, 0, 0.2)),
        "red": ((0.9098, 0.2471, 0.2824), (0, 0.86, 0.65, 0)),
        "green": ((0, 0.5333, 0.2078), (0.89, 0.05, 1, 0.17)),
        "purple": ((0.4745, 0.1373, 0.5569), (0.67, 0.96, 0, 0)),
    }

    if format not in {"rgb", "cmyk", "hex"}:
        raise ValueError("Invalid format. Choose 'rgb', 'cmyk', or 'hex'.")

    def format_color(color):
        if format == "rgb":
            return color[0]
        elif format == "cmyk":
            return color[1]
        elif format == "hex":
            return rgb_to_hex(color[0])

    if color_name:
        color = palette.get(color_name)
        if not color:
            return {color_name: "Color not found"}
        return format_color(color)

    return {name: format_color(values) for name, values in palette.items()}


def plot_leading_edge_profile(
    image, profile, ranges, beams, threshold, color="b", fig_size=(12, 6)
):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=fig_size)
    # Plot the sonar image
    im = ax1.imshow(image, cmap="gray", vmin=0, vmax=255)

    # Create an axis for the colorbar that matches the height of the image axis
    divider = make_axes_locatable(ax1)
    cax = divider.append_axes("right", size="10%", pad=0.05)
    fig.colorbar(im, cax=cax, orientation="vertical")

    num_beams = image.shape[1]
    leading_edges = _get_leading_edges(image >= threshold)

    # Filter out zero values
    valid_indices = np.where(leading_edges > 0)
    valid_beams = np.array(range(num_beams))[valid_indices]
    valid_edges = leading_edges[valid_indices]

    # Plot the valid points on the sonar image
    plot_sonar_image(ax1, image, ranges, beams, "Sonar Image with Leading Edges")
    ax1.scatter(valid_beams, valid_edges, c=color, s=2)

    # Plot the x and y coordinates on the second subplot
    ax2.scatter(profile.y, profile.x, c=color, s=1.5)
    ax2.set_xlabel("Y [m]")
    ax2.set_ylabel("X [m]")
    ax2.set_title("YX Coordinates")
    x_min = ranges[-1] * np.sin(beams[0])
    x_max = -x_min
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(ranges[0], ranges[-1])
    ax2.set_aspect("equal")
    ax2.grid(True)

    return fig, (ax1, ax2)


def plot_RANSAC(profile, regressor, ax):
    x = profile.x
    y = profile.y
    inlier_error = np.abs(
        regressor.predict(y[profile.valid].reshape(-1, 1))
        - x[profile.valid]
    )
    ax.plot(
        y,
        regressor.predict(y.reshape(-1, 1)),
        color=get_dtu_color_palette("yellow"),
        label="RANSAC \n|x - f(y)| = {:.2f}".format(np.mean(inlier_error)),
        linewidth=1,
    )
    ax.scatter(
        y[profile.valid],
        x[profile.valid],
        color=get_dtu_color_palette("green"),
        label="Valid",
        s=3,
    )
    ax.scatter(
        y[~profile.valid],
        x[~profile.valid],
        color=get_dtu_color_palette("red"),
        label="Invalid",
        s=3,
    )

    ax.set_xlabel("y [m]")
    ax.set_ylabel("x [m]")
    ax.set_aspect("equal")
    # Top left corner legend
    ax.legend(loc="upper left")

def plot_MAD(profile, upper_bound, lower_bound, ax):
    x = profile.x
    y = profile.y
    ax.plot(
        y,
        upper_bound,
        "--",
        color=get_dtu_color_palette("yellow"),
        label="Upper bound",
        linewidth=1,
    )
    ax.plot(
        y,
        lower_bound,
        "--",
        color=get_dtu_color_palette("orange"),
        label="Lower bound",
        linewidth=1,
    )
    ax.scatter(
        y[profile.valid],
        x[profile.valid],
        color=get_dtu_color_palette("green"),
        label="Valid",
        s=3,
    )
    ax.scatter(
        y[~profile.valid],
        x[~profile.valid],
        color=get_dtu_color_palette("red"),
        label="Invalid",
        s=3,
    )
    ax.legend()
    ax.set_xlabel("y [m]")
    ax.set_ylabel("x [m]")


def create3d_environment(pool_structure: dict, pool_structure_vertices: dict, walls=True):

    # Create a figure with 4 subplots
    fig_size = set_size("thesis", height_ratio=1.5, fraction=1)
    fig = plt.figure(figsize=fig_size)
    cylinder_color = 'orange'
    wall_color = 'black'

    # XY plane
    ax1 = fig.add_subplot(221)
    ax1.plot(pool_structure['cylinder1'][0][0], pool_structure['cylinder1'][1][0], color=cylinder_color, alpha=0.8)
    ax1.plot(pool_structure['cylinder2'][0][0], pool_structure['cylinder2'][1][0], color=cylinder_color, alpha=0.8)
    ax1.plot(pool_structure['left_wall'][:,0], pool_structure['left_wall'][:,1], color=wall_color)
    ax1.plot(pool_structure['right_wall'][:,0], pool_structure['right_wall'][:,1], color=wall_color)
    ax1.plot(pool_structure['end_wall'][:,0], pool_structure['end_wall'][:,1], color=wall_color)
    ax1.plot(pool_structure['home_wall'][:,0], pool_structure['home_wall'][:,1], color=wall_color)
    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('y [m]')
    ax1.set_title('a) XY Plane')
    ax1.set_aspect('equal')
    ax1.grid(True)

    # XZ plane
    ax2 = fig.add_subplot(222)
    ax2.plot(pool_structure['cylinder1'][0], pool_structure['cylinder1'][2][:, 0], color=cylinder_color, alpha=0.1)
    ax2.plot(pool_structure['cylinder2'][0], pool_structure['cylinder2'][2], color=cylinder_color, alpha=0.1)
    ax2.plot(pool_structure['floor'][:,0], pool_structure['floor'][:,2], color=wall_color)
    ax2.plot(pool_structure['end_wall'][:,0], pool_structure['end_wall'][:,2], color=wall_color)
    ax2.set_xlabel('x [m]')
    ax2.set_ylabel('z [m]')
    ax2.set_title('b) XZ Plane')
    ax2.set_aspect('equal')
    ax2.grid(True)

    # YZ plane
    ax3 = fig.add_subplot(223)
    ax3.plot(pool_structure['cylinder1'][1], pool_structure['cylinder1'][2][:, 0], color=cylinder_color, alpha=0.1)
    ax3.plot(pool_structure['cylinder2'][1], pool_structure['cylinder2'][2][:, 0], color=cylinder_color, alpha=0.1)
    ax3.plot(pool_structure['floor'][:,1], pool_structure['floor'][:,2], color=wall_color)
    ax3.plot(pool_structure['end_wall'][:,1], pool_structure['end_wall'][:,2], color=wall_color)
    ax3.plot(pool_structure['home_wall'][:,1], pool_structure['home_wall'][:,2], color=wall_color)
    ax3.set_xlabel('y [m]')
    ax3.set_ylabel('z [m]')
    ax3.set_title('c) YZ Plane')
    ax3.set_aspect('equal')
    ax3.grid(True)

    # 3D plot
    ax4 = fig.add_subplot(224, projection='3d')
    ax4.set_xlabel('x [m]')
    ax4.set_ylabel('y [m]')
    ax4.set_zlabel('z [m]')
    ax4.set_title('d) 3D Plot')
    ax4.plot_surface(pool_structure['cylinder1'][0], pool_structure['cylinder1'][1], pool_structure['cylinder1'][2], color=cylinder_color, alpha=0.3, linewidth=0)
    ax4.plot_surface(pool_structure['cylinder2'][0], pool_structure['cylinder2'][1], pool_structure['cylinder2'][2], color=cylinder_color, alpha=0.3, linewidth=0)
        
    # Add walls and floor to the 3D plot
    left_wall_poly = Poly3DCollection(pool_structure_vertices['left_wall'], alpha=0.1, facecolor=wall_color)
    right_wall_poly = Poly3DCollection(pool_structure_vertices['right_wall'], alpha=0.1, facecolor=wall_color)
    end_wall_poly = Poly3DCollection(pool_structure_vertices['end_wall'], alpha=0.1, facecolor=wall_color)
    floor_poly = Poly3DCollection(pool_structure_vertices['floor'], alpha=0.3, facecolor=wall_color)
    
    if walls:
        ax4.add_collection3d(left_wall_poly)
        ax4.add_collection3d(right_wall_poly)
        ax4.add_collection3d(end_wall_poly)
        ax4.add_collection3d(floor_poly)



    legend_elements = [Line2D([0], [0], color='black', lw=2, label='Floor and Walls'),
                    Patch(facecolor=cylinder_color, alpha=0.5, label='Cylinders')]

    plt.tight_layout()
    fig.legend(handles=legend_elements, loc='lower center', bbox_to_anchor=(0.5, -0.1), ncol=3)

    return fig, [ax1, ax2, ax3, ax4]

def plot_points_2x2(points, color, axs, point_size=1):
    axs[0].scatter(points[0], points[1], color=color, s=point_size)
    axs[1].scatter(points[0], points[2], color=color, s=point_size)
    axs[2].scatter(points[1], points[2], color=color, s=point_size)
    axs[3].scatter(points[0], points[1], points[2], color=color, s=point_size)
    return axs

def plot_environment_on_axis(ax, pool_structure, pool_structure_vertices, plane="XY", walls=True):
    cylinder_color = 'orange'
    wall_color = 'black'

    if plane == "XY":
        # XY Plane
        ax.plot(pool_structure['cylinder1'][0][0], pool_structure['cylinder1'][1][0], color=cylinder_color, alpha=0.8)
        ax.plot(pool_structure['cylinder2'][0][0], pool_structure['cylinder2'][1][0], color=cylinder_color, alpha=0.8)
        ax.plot(pool_structure['left_wall'][:,0], pool_structure['left_wall'][:,1], color=wall_color)
        ax.plot(pool_structure['right_wall'][:,0], pool_structure['right_wall'][:,1], color=wall_color)
        ax.plot(pool_structure['end_wall'][:,0], pool_structure['end_wall'][:,1], color=wall_color)
        ax.plot(pool_structure['home_wall'][:,0], pool_structure['home_wall'][:,1], color=wall_color)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('XY Plane')
        ax.set_aspect('equal')
        ax.grid(True)

    elif plane == "XZ":
        # XZ Plane
        ax.plot(pool_structure['cylinder1'][0], pool_structure['cylinder1'][2][:, 0], color=cylinder_color, alpha=0.1)
        ax.plot(pool_structure['cylinder2'][0], pool_structure['cylinder2'][2], color=cylinder_color, alpha=0.1)
        ax.plot(pool_structure['floor'][:,0], pool_structure['floor'][:,2], color=wall_color)
        ax.plot(pool_structure['end_wall'][:,0], pool_structure['end_wall'][:,2], color=wall_color)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('z [m]')
        ax.set_title('XZ Plane')
        ax.set_aspect('equal')
        ax.grid(True)

    elif plane == "YZ":
        # YZ Plane
        ax.plot(pool_structure['cylinder1'][1], pool_structure['cylinder1'][2][:, 0], color=cylinder_color, alpha=0.1)
        ax.plot(pool_structure['cylinder2'][1], pool_structure['cylinder2'][2][:, 0], color=cylinder_color, alpha=0.1)
        ax.plot(pool_structure['floor'][:,1], pool_structure['floor'][:,2], color=wall_color)
        ax.plot(pool_structure['end_wall'][:,1], pool_structure['end_wall'][:,2], color=wall_color)
        ax.plot(pool_structure['home_wall'][:,1], pool_structure['home_wall'][:,2], color=wall_color)
        ax.set_xlabel('y [m]')
        ax.set_ylabel('z [m]')
        ax.set_title('YZ Plane')
        ax.set_aspect('equal')
        ax.grid(True)

    elif plane == "XYZ":
        # 3D Plot
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.set_title('3D Plot')
        ax.plot_surface(pool_structure['cylinder1'][0], pool_structure['cylinder1'][1], pool_structure['cylinder1'][2], 
                        color=cylinder_color, alpha=0.3, linewidth=0)
        ax.plot_surface(pool_structure['cylinder2'][0], pool_structure['cylinder2'][1], pool_structure['cylinder2'][2], 
                        color=cylinder_color, alpha=0.3, linewidth=0)

        # Add walls and floor to the 3D plot
        if walls:
            left_wall_poly = Poly3DCollection(pool_structure_vertices['left_wall'], alpha=0.1, facecolor=wall_color)
            right_wall_poly = Poly3DCollection(pool_structure_vertices['right_wall'], alpha=0.1, facecolor=wall_color)
            end_wall_poly = Poly3DCollection(pool_structure_vertices['end_wall'], alpha=0.1, facecolor=wall_color)
            floor_poly = Poly3DCollection(pool_structure_vertices['floor'], alpha=0.3, facecolor=wall_color)
            ax.add_collection3d(left_wall_poly)
            ax.add_collection3d(right_wall_poly)
            ax.add_collection3d(end_wall_poly)
            ax.add_collection3d(floor_poly)

    else:
        raise ValueError("Invalid plane specified. Use 'XY', 'XZ', 'YZ', or 'XYZ'.")

