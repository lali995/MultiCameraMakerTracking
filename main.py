import json
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objects as go

def get_camera_poses(config_path='config'):
    """
    Finds all sensor_config.json files in the config directory and extracts the
    camera's position and direction.

    Args:
        config_path (str): The path to the config directory.

    Returns:
        list: A list of dictionaries, each containing 'position', 'direction', and 'name'.
    """
    camera_poses = []
    config_folders = glob.glob(os.path.join(config_path, 'SZVIDS-*'))
    for folder in config_folders:
        sensor_config_path = os.path.join(folder, 'sensor_config.json')
        if os.path.exists(sensor_config_path):
            with open(sensor_config_path, 'r') as f:
                try:
                    config_data = json.load(f)
                    extrinsic = config_data.get('Sensor to World Extrinsic')
                    if extrinsic:
                        extrinsic_matrix = np.array(extrinsic).reshape(4, 4)
                        # Position is the translation vector (last column)
                        position = extrinsic_matrix[:3, 3]
                        # Direction: the 3rd column of rotation matrix is the camera's Z-axis in world coords
                        # This sensor uses OpenCV convention where camera looks down +Z
                        direction = extrinsic_matrix[:3, 2]
                        # Get folder name as camera identifier
                        folder_name = os.path.basename(folder)
                        camera_poses.append({
                            'position': position.tolist(),
                            'direction': direction.tolist(),
                            'name': folder_name
                        })
                except json.JSONDecodeError:
                    print(f"Error decoding JSON from {sensor_config_path}")
    return camera_poses

def render_scene(ax, camera_poses):
    """
    Renders a room-like box, camera positions, and their view directions.

    Args:
        ax (Axes3D): The 3D axes to plot on.
        camera_poses (list): A list of dictionaries with camera pose information.
    """
    if not camera_poses:
        print("No camera poses to render.")
        return

    positions = np.array([pose['position'] for pose in camera_poses])
    directions = np.array([pose['direction'] for pose in camera_poses])

    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
    u, v, w = directions[:, 0], directions[:, 1], directions[:, 2]

    # Plot camera positions
    ax.scatter(x, y, z, c='r', marker='o', label='Cameras')

    # Plot camera view directions as arrows (cones)
    ax.quiver(x, y, z, u, v, w, length=0.5, normalize=True, color='b', label='View Direction')

    # Create a bounding box for the room, centered at the origin
    all_coords = positions.flatten()
    if all_coords.size == 0:
        max_abs_coord = 0.5 # Default range if no cameras
    else:
        max_abs_coord = np.max(np.abs(all_coords))

    # Ensure the plot limits are symmetrical around 0 with some padding
    padding = 0.5
    plot_limit = max_abs_coord + padding

    ax.set_xlim(-plot_limit, plot_limit)
    ax.set_ylim(-plot_limit, plot_limit)
    ax.set_zlim(-plot_limit, plot_limit)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title('Camera Poses in Room')

def render_scene_interactive(camera_poses):
    """
    Renders an interactive 3D visualization using Plotly.
    Supports zoom, pan, rotate, and hover inspection.

    Args:
        camera_poses (list): A list of dictionaries with camera pose information.
    """
    if not camera_poses:
        print("No camera poses to render.")
        return

    positions = np.array([pose['position'] for pose in camera_poses])
    directions = np.array([pose['direction'] for pose in camera_poses])
    names = [pose['name'] for pose in camera_poses]

    fig = go.Figure()

    # Plot camera positions as markers
    fig.add_trace(go.Scatter3d(
        x=positions[:, 0],
        y=positions[:, 1],
        z=positions[:, 2],
        mode='markers',
        marker=dict(size=8, color='red'),
        name='Cameras',
        text=names,
        hovertemplate='<b>%{text}</b><br>X: %{x:.3f}<br>Y: %{y:.3f}<br>Z: %{z:.3f}<extra></extra>'
    ))

    # Plot view direction arrows using Cone
    # Scale arrow length based on scene size
    arrow_length = 0.3
    fig.add_trace(go.Cone(
        x=positions[:, 0],
        y=positions[:, 1],
        z=positions[:, 2],
        u=directions[:, 0],
        v=directions[:, 1],
        w=directions[:, 2],
        sizemode='absolute',
        sizeref=arrow_length,
        anchor='tail',
        colorscale=[[0, 'blue'], [1, 'blue']],
        showscale=False,
        name='View Direction'
    ))

    # Add lines from camera to arrow tip for better visibility
    for i in range(len(positions)):
        end_point = positions[i] + directions[i] * arrow_length * 2
        fig.add_trace(go.Scatter3d(
            x=[positions[i, 0], end_point[0]],
            y=[positions[i, 1], end_point[1]],
            z=[positions[i, 2], end_point[2]],
            mode='lines',
            line=dict(color='blue', width=3),
            showlegend=False,
            hoverinfo='skip'
        ))

    # Calculate scene bounds
    all_coords = positions.flatten()
    if all_coords.size == 0:
        max_abs_coord = 1.0
    else:
        max_abs_coord = np.max(np.abs(all_coords))

    padding = 0.5
    plot_limit = max_abs_coord + padding

    # Add origin marker
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode='markers',
        marker=dict(size=6, color='black', symbol='cross'),
        name='Origin',
        hovertemplate='<b>Origin</b><br>X: 0<br>Y: 0<br>Z: 0<extra></extra>'
    ))

    # Update layout for better visualization
    # Y-axis up, Z-axis front to back, X-axis left to right
    fig.update_layout(
        title='Camera Poses - Interactive View',
        scene=dict(
            xaxis=dict(range=[-plot_limit, plot_limit], title='X (left-right)'),
            yaxis=dict(range=[-plot_limit, plot_limit], title='Y (up)'),
            zaxis=dict(range=[-plot_limit, plot_limit], title='Z (front-back)'),
            aspectmode='cube',
            camera=dict(
                up=dict(x=0, y=1, z=0),  # Y is up
                center=dict(x=0, y=0, z=0),
                eye=dict(x=1.5, y=1.5, z=1.5)  # Initial view position
            )
        ),
        showlegend=True,
        legend=dict(x=0.02, y=0.98),
        margin=dict(l=0, r=0, t=40, b=0)
    )

    fig.show()


if __name__ == '__main__':
    import sys

    poses = get_camera_poses()

    # Check command line argument for visualization mode
    if len(sys.argv) > 1 and sys.argv[1] == '--matplotlib':
        # Use matplotlib (original)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        render_scene(ax, poses)
        plt.show()
    else:
        # Use interactive Plotly (default)
        render_scene_interactive(poses)
