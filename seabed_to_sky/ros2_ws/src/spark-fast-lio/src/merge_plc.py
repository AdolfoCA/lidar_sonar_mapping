#!/usr/bin/env python3
"""
Fuse LiDAR and Sonar Point Clouds using saved pose metadata
"""

import numpy as np
import open3d as o3d
import json
from pathlib import Path


def read_pcd(filepath):
    """Read PCD file using Open3D and return points and intensity"""
    pcd = o3d.io.read_point_cloud(filepath)
    points = np.asarray(pcd.points)
    
    if pcd.colors is not None and len(pcd.colors) > 0:
        colors = np.asarray(pcd.colors)
        intensity = np.mean(colors, axis=1) * 255
    else:
        intensity = np.ones(len(points)) * 100.0
    
    # Remove NaN and Inf values
    valid_mask = ~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)
    points_clean = points[valid_mask]
    intensity_clean = intensity[valid_mask]
    
    removed = len(points) - len(points_clean)
    if removed > 0:
        print(f'  Read {len(points)} points, removed {removed} invalid (NaN/Inf)')
    else:
        print(f'  Read {len(points)} points')
    
    return points_clean, intensity_clean


def write_pcd(filepath, points, intensity, source_labels):
    """Write points to PCD file (ASCII format, MATLAB compatible)"""
    n_points = len(points)
    
    with open(filepath, 'w') as f:
        f.write('# .PCD v.7 - Point Cloud Data file format\n')
        f.write('VERSION .7\n')
        f.write('FIELDS x y z intensity source\n')
        f.write('SIZE 4 4 4 4 4\n')
        f.write('TYPE F F F F F\n')
        f.write('COUNT 1 1 1 1 1\n')
        f.write(f'WIDTH {n_points}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {n_points}\n')
        f.write('DATA ascii\n')
        
        for i in range(n_points):
            f.write(f'{points[i, 0]:.6f} {points[i, 1]:.6f} {points[i, 2]:.6f} '
                    f'{intensity[i]:.6f} {source_labels[i]:.0f}\n')


def fuse_pointclouds(lidar_file, sonar_file, output_file, metadata_file=None):
    """
    Fuse LiDAR and sonar point clouds using initial pose alignment
    """
    
    print(f'Loading LiDAR: {lidar_file}')
    lidar_pts, lidar_intensity = read_pcd(lidar_file)
    
    print(f'Loading Sonar: {sonar_file}')
    sonar_pts, sonar_intensity = read_pcd(sonar_file)
    
    if len(lidar_pts) == 0 or len(sonar_pts) == 0:
        print('ERROR: No valid points!')
        return
    
    print(f'\nValid points:')
    print(f'  LiDAR: {len(lidar_pts)}')
    print(f'  Sonar: {len(sonar_pts)}')
    
    # Determine alignment offset
    if metadata_file and Path(metadata_file).exists():
        # Use saved initial poses
        print(f'\nUsing metadata: {metadata_file}')
        with open(metadata_file, 'r') as f:
            metadata = json.load(f)
        
        lidar_init = metadata.get('lidar_initial_pose')
        sonar_init = metadata.get('sonar_initial_pose')
        
        if lidar_init and sonar_init:
            lidar_xy_origin = np.array([lidar_init['x'], lidar_init['y']])
            sonar_xy_origin = np.array([sonar_init['x'], sonar_init['y']])
            print(f'  LiDAR initial pose: [{lidar_xy_origin[0]:.3f}, {lidar_xy_origin[1]:.3f}]')
            print(f'  Sonar initial pose: [{sonar_xy_origin[0]:.3f}, {sonar_xy_origin[1]:.3f}]')
        else:
            print('  WARNING: Initial poses not found in metadata, using centroid alignment')
            lidar_xy_origin = np.mean(lidar_pts[:, :2], axis=0)
            sonar_xy_origin = np.mean(sonar_pts[:, :2], axis=0)
    else:
        # Fallback: Use centroid alignment (better than first point for unordered data)
        print('\nNo metadata file, using centroid alignment')
        lidar_xy_origin = np.mean(lidar_pts[:, :2], axis=0)
        sonar_xy_origin = np.mean(sonar_pts[:, :2], axis=0)
        print(f'  LiDAR centroid: [{lidar_xy_origin[0]:.3f}, {lidar_xy_origin[1]:.3f}]')
        print(f'  Sonar centroid: [{sonar_xy_origin[0]:.3f}, {sonar_xy_origin[1]:.3f}]')
    
    # Calculate and apply offset
    xy_offset = lidar_xy_origin - sonar_xy_origin
    
    sonar_pts_aligned = sonar_pts.copy()
    sonar_pts_aligned[:, 0] += xy_offset[0]
    sonar_pts_aligned[:, 1] += xy_offset[1]
    
    print(f'\nApplied X-Y offset: [{xy_offset[0]:.3f}, {xy_offset[1]:.3f}]')
    
    print(f'\nZ ranges (Z is up):')
    print(f'  LiDAR Z: [{lidar_pts[:, 2].min():.3f}, {lidar_pts[:, 2].max():.3f}] m')
    print(f'  Sonar Z: [{sonar_pts_aligned[:, 2].min():.3f}, {sonar_pts_aligned[:, 2].max():.3f}] m')
    
    # Fuse point clouds
    fused_pts = np.vstack([lidar_pts, sonar_pts_aligned])
    fused_intensity = np.hstack([lidar_intensity, sonar_intensity])
    
    source_labels = np.hstack([
        np.ones(len(lidar_pts)),
        np.full(len(sonar_pts_aligned), 2)
    ])
    
    # Save
    write_pcd(output_file, fused_pts, fused_intensity, source_labels)
    
    # Statistics
    print('\n' + '=' * 50)
    print('FUSED POINT CLOUD')
    print('=' * 50)
    print(f'Total points: {len(fused_pts)}')
    print(f'  LiDAR: {len(lidar_pts)} ({100 * len(lidar_pts) / len(fused_pts):.1f}%)')
    print(f'  Sonar: {len(sonar_pts)} ({100 * len(sonar_pts) / len(fused_pts):.1f}%)')
    print(f'\nBounding box:')
    print(f'  X: [{fused_pts[:, 0].min():.3f}, {fused_pts[:, 0].max():.3f}] m')
    print(f'  Y: [{fused_pts[:, 1].min():.3f}, {fused_pts[:, 1].max():.3f}] m')
    print(f'  Z: [{fused_pts[:, 2].min():.3f}, {fused_pts[:, 2].max():.3f}] m')
    print(f'\nSaved: {output_file}')


def main():
    # Set your file paths here
    lidar_file = '/home/rosdev/spark-fast-lio/saved_maps/lidar_map.pcd'
    sonar_file = '/home/rosdev/spark-fast-lio/saved_maps/sonar_map.pcd'
    output_file = '/home/rosdev/spark-fast-lio/saved_maps/fused.pcd'
    metadata_file = '/home/rosdev/spark-fast-lio/saved_maps/metadata.json'
    
    fuse_pointclouds(lidar_file, sonar_file, output_file, metadata_file)


if __name__ == '__main__':
    main()