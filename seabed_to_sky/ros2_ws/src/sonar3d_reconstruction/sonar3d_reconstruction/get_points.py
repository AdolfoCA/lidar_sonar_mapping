import numpy as np 
from scipy.spatial.transform import Rotation as R
from sonar3d_reconstruction.process_sonar_image import (
    ProjectedSonarImage2Image, 
    cut_image, 
    filter_image_rows, 
    polar2cartesian, 
)
from sonar3d_reconstruction.patch_detection import (
    features_to_XYI, 
    cluster_features, 
    cluster_descriptors, 
    match_clusters, 
    match_within_clusters, 
    extract_features
)
from sonar3d_reconstruction.sonar_to_pointcloud import populate_profile
from cfar.CFAR import CFAR
from sonar3d_reconstruction.process_sonar_image import (
    filter_horizontal_image,
    filter_vertical_image,
)
from sonar3d_reconstruction.edge_detection import image2Profile

def df_row_to_matrix(row):
    translation = np.array([row.x, row.y, row.z])
    rotation = np.array([row.qx, row.qy, row.qz, row.qw])

    # Create 4x4 matrix using the rotation and translation
    rotation_matrix = R.from_quat(rotation).as_matrix()
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = translation

    return matrix

def compute_transforms(calib_offset_oculus, calib_offset_blueview, theta_z, tilt_angle):
    # Small rotation around Z-axis of pole
    theta = np.deg2rad(theta_z)

    # 1. Top of the Pole to Joint Point
    T_pole_top_to_joint = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, -2.28],
        [0, 0, 0, 1]
    ])

    # 2. Joint Point Tilt (-40 degrees around X-axis)
    theta = np.deg2rad(-tilt_angle)
    R_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

    # Combined transform to joint
    T_pole_top_to_joint_tilted = T_pole_top_to_joint @ R_x

    # 3. Horizontal Sonar to Joint Point
    T_joint_to_horizontal = np.array([
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.21 + calib_offset_oculus],
        [0, 0, 1, -0.05],
        [0, 0, 0, 1]
    ])

    # Horizontal Sonar to Top of Pole
    T_horizontal_to_pole_top = T_pole_top_to_joint_tilted @ T_joint_to_horizontal

    # 4. Vertical Sonar to Joint Point
    T_joint_to_vertical = np.array([
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.26 + calib_offset_blueview],
        [0, 0, 1, 0.1],
        [0, 0, 0, 1]
    ])

    # Rotation 90 degrees around Y-axis for Vertical Sonar
    R_y_90 = R.from_euler('y', 90, degrees=True).as_matrix()
    R_y_90_homogeneous = np.eye(4)
    R_y_90_homogeneous[:3, :3] = R_y_90

    # Vertical Sonar to Top of Pole
    T_vertical_to_pole_top = T_pole_top_to_joint_tilted @ T_joint_to_vertical @ R_y_90_homogeneous

    return T_horizontal_to_pole_top, T_vertical_to_pole_top


# 3D patch points    
def invert_transform(T):
    R_inv = T[:3, :3].T  # Transpose of the rotation part
    t_inv = -R_inv @ T[:3, 3]  # Inverted translation
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

def get_all_points(sonars_df, params_horizontal, params_vertical, params, transform_matrix_oculus, transform_matrix_blueview, oculus_linescan_pitch: float = 0.0):    
    for i in range(len(sonars_df)):
        # Sonar data
        SonarImageHorizontal = sonars_df['oculus_image'].iloc[i]
        SonarImageVertical = sonars_df['blueview_image'].iloc[i]

        imageHorizontal, rangesHorizontal, beamsHorizontal = ProjectedSonarImage2Image(SonarImageHorizontal)
        imageVertical, rangesVertical, beamsVertical = ProjectedSonarImage2Image(SonarImageVertical)
        sound_speed_horizontal = SonarImageHorizontal.ping_info.sound_speed
        sound_speed_vertical = SonarImageVertical.ping_info.sound_speed
        rangesVertical *= sound_speed_horizontal / sound_speed_vertical

        start_range_idx = np.where(rangesHorizontal >= params_horizontal['sonar']['min_range'])[0][0]    
        rangesHorizontal = rangesHorizontal[start_range_idx:]
        imageHorizontal = imageHorizontal[start_range_idx:, :]

        start_range_idx = np.where(rangesVertical >= params_vertical['sonar']['min_range'])[0][0]
        rangesVertical = rangesVertical[start_range_idx:]
        imageVertical = imageVertical[start_range_idx:, :]

        profileHorizontal = image2Profile(imageHorizontal, rangesHorizontal, beamsHorizontal, params_horizontal['sonar']['threshold'], pitch=np.deg2rad(oculus_linescan_pitch))
        profileVertical = image2Profile(imageVertical, rangesVertical, beamsVertical, params_vertical['sonar']['threshold'])

        # Filter the image
        imageHorizontal = filter_horizontal_image(imageHorizontal, method = []) # The gain is low, so we don't need to filter with otsu
        imageVertical = filter_vertical_image(imageVertical)

        # Create an interactive 3D scatter plot using Plotly
        profileHorizontal = profileHorizontal.filter_valid()
        profileVertical = profileVertical.filter_valid()

        # Make point arrays
        pointsHorizontal = np.vstack((-profileHorizontal.y, profileHorizontal.x, -profileHorizontal.z, np.ones_like(profileHorizontal.x)))
        pointsVertical = np.vstack((-profileVertical.y, profileVertical.x, -profileVertical.z, np.ones_like(profileVertical.x)))

        # Point are in sonar frame. The have to be transformed pole_mount_top frame and then to the optitrack frame
        pointsHorizontal = np.dot(transform_matrix_oculus, pointsHorizontal)
        pointsVertical = np.dot(transform_matrix_blueview, pointsVertical)

        pointsHorizontal = np.dot(df_row_to_matrix(sonars_df.iloc[i]), pointsHorizontal)
        pointsVertical = np.dot(df_row_to_matrix(sonars_df.iloc[i]), pointsVertical)

        if i == 0:
            all_horizontal_points = pointsHorizontal
            all_vertical_points = pointsVertical
        else:
            all_horizontal_points = np.hstack((all_horizontal_points, pointsHorizontal))
            all_vertical_points = np.hstack((all_vertical_points, pointsVertical))


    T_horizontal_inv = invert_transform(transform_matrix_oculus)
    T_vertical_to_horizontal = T_horizontal_inv @ transform_matrix_blueview
    rel_transform_linear = T_vertical_to_horizontal[:3, 3]
    rel_transform_rotation = R.from_matrix(T_vertical_to_horizontal[:3, :3])

    detector_horizontal = CFAR(params['cfar']['horizontal']['tc'], 
                               params['cfar']['horizontal']['gc'], 
                               params['cfar']['horizontal']['pfa'], None)
    detector_vertical = CFAR(params['cfar']['vertical']['tc'], 
                             params['cfar']['vertical']['gc'], 
                             params['cfar']['vertical']['pfa'], None)
    for sonar_index in range(len(sonars_df)):
        SonarImageHorizontal = sonars_df['oculus_image'].iloc[sonar_index]
        SonarImageVertical = sonars_df['blueview_image'].iloc[sonar_index]

        sound_speed_horizontal = SonarImageHorizontal.ping_info.sound_speed
        sound_speed_vertical = SonarImageVertical.ping_info.sound_speed
        rangesVertical *= sound_speed_horizontal / sound_speed_vertical
        imageHorizontal, rangesHorizontal, beamsHorizontal = ProjectedSonarImage2Image(SonarImageHorizontal)
        imageVertical, rangesVertical, beamsVertical = ProjectedSonarImage2Image(SonarImageVertical)

        # Cut the image pair for alignment
        desired_aperture = np.deg2rad(20)
        imageHorizontal, rangesHorizontal, beamsHorizontal = cut_image(imageHorizontal, desired_aperture, beamsHorizontal, rangesHorizontal, 0.0)
        imageVertical, rangesVertical, beamsVertical = cut_image(imageVertical, desired_aperture, beamsVertical, rangesVertical, 0.0)

        # 2D array of x coordinate in sonar fame
        xHorizontal, yHorizontal = polar2cartesian(rangesHorizontal, beamsHorizontal)
        xVertical, yVertical = polar2cartesian(rangesVertical, beamsVertical, rel_transform_linear[0], rel_transform_linear[2])

        # Filter rows based on x coordinates
        imageHorizontal, rangesHorizontal, xHorizontal, yHorizontal = filter_image_rows(imageHorizontal, rangesHorizontal, xHorizontal, xVertical, yHorizontal)
        imageVertical, rangesVertical, xVertical, yVertical = filter_image_rows(imageVertical, rangesVertical, xVertical, xHorizontal, yVertical)

        # Filter the image
        imageHorizontal = filter_horizontal_image(imageHorizontal)
        imageVertical = filter_vertical_image(imageVertical)
        
        horizontalFeatures = extract_features(imageHorizontal, detector_horizontal, "SOCA", params['cfar']['horizontal']['threshold'])
        verticalFeatures = extract_features(imageVertical, detector_vertical, "SOCA", params['cfar']['vertical']['threshold'])

        if len(horizontalFeatures) == 0 or len(verticalFeatures) == 0:
            continue

        horizontalXYI = features_to_XYI(horizontalFeatures, imageHorizontal, xHorizontal, yHorizontal)
        verticalXYI = features_to_XYI(verticalFeatures, imageVertical, xVertical, yVertical)

        horizontalLabels, horizontalXYI = cluster_features(horizontalXYI, params['cluster']['threshold'], params['cluster']['min_samples'])
        verticalLabels, verticalXYI = cluster_features(verticalXYI, params['cluster']['threshold'], params['cluster']['min_samples'])

        horizontalDescriptors = cluster_descriptors(horizontalXYI, horizontalLabels)
        verticalDescriptors = cluster_descriptors(verticalXYI, verticalLabels)
        CH_Matches, CV_Matches = match_clusters(horizontalDescriptors, verticalDescriptors)

        roll_vertical_deg =np.rad2deg(rel_transform_rotation.as_euler('xyz')[0])
        matches = match_within_clusters(CH_Matches, CV_Matches, 
                            horizontalXYI, verticalXYI,
                            horizontalLabels, verticalLabels,
                            imageHorizontal, imageVertical,
                            rot_vertical=roll_vertical_deg)

        uncertaintyMax = params['uncertaintyMax']
        profile = populate_profile(matches,
                                   uncertaintyMax)
        try:
            profile = profile.filter_valid()
        except IndexError:
            print("No valid points in profile")
        # Make point arrays
        points = np.vstack((-profile.y, profile.x, -profile.z, np.ones_like(profile.x)))
        
        points = np.dot(transform_matrix_oculus, points)

        points = np.dot(df_row_to_matrix(sonars_df.iloc[sonar_index]), points)
        
        if sonar_index == 0:
            all_patch_points = points
        else:
            all_patch_points = np.hstack((all_patch_points, points))

    return all_horizontal_points, all_vertical_points, all_patch_points