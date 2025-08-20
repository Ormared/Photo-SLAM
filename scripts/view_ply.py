#!/usr/bin/env python3
"""
Simple Python script to visualize PLY files from Photo-SLAM results
Requires: pip install open3d matplotlib numpy
"""

import open3d as o3d
import numpy as np
import argparse
import sys

def visualize_ply(ply_path):
    """
    Visualize a PLY file using Open3D
    """
    try:
        # Load the point cloud
        print(f"Loading PLY file: {ply_path}")
        pcd = o3d.io.read_point_cloud(ply_path)
        
        if len(pcd.points) == 0:
            print("Error: No points found in the PLY file")
            return False
            
        print(f"Loaded {len(pcd.points)} points")
        
        # Print basic statistics
        points = np.asarray(pcd.points)
        print(f"Point cloud bounds:")
        print(f"  X: [{np.min(points[:, 0]):.3f}, {np.max(points[:, 0]):.3f}]")
        print(f"  Y: [{np.min(points[:, 1]):.3f}, {np.max(points[:, 1]):.3f}]")
        print(f"  Z: [{np.min(points[:, 2]):.3f}, {np.max(points[:, 2]):.3f}]")
        
        # Check if colors exist
        if len(pcd.colors) > 0:
            print("Point cloud has colors")
        else:
            print("Point cloud has no colors, using default")
            # Color points by height (Z coordinate)
            colors = np.zeros_like(points)
            z_normalized = (points[:, 2] - np.min(points[:, 2])) / (np.max(points[:, 2]) - np.min(points[:, 2]))
            colors[:, 0] = z_normalized  # Red channel
            colors[:, 2] = 1 - z_normalized  # Blue channel
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Visualize
        print("\nStarting visualization...")
        print("Controls:")
        print("  - Mouse: Rotate view")
        print("  - Mouse wheel: Zoom")
        print("  - Ctrl+Mouse: Pan")
        print("  - Q or ESC: Quit")
        
        # Create visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Photo-SLAM Point Cloud", width=1200, height=800)
        vis.add_geometry(pcd)
        
        # Set nice viewing options
        opt = vis.get_render_option()
        opt.point_size = 2.0
        opt.background_color = np.asarray([0.1, 0.1, 0.1])
        
        # Run visualization
        vis.run()
        vis.destroy_window()
        
        return True
        
    except Exception as e:
        print(f"Error visualizing PLY file: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Visualize Photo-SLAM PLY results")
    parser.add_argument("ply_file", help="Path to the PLY file")
    parser.add_argument("--downsample", type=float, default=None, 
                       help="Downsample ratio (e.g., 0.1 for 10%% of points)")
    
    args = parser.parse_args()
    
    if not visualize_ply(args.ply_file):
        sys.exit(1)

if __name__ == "__main__":
    main()
