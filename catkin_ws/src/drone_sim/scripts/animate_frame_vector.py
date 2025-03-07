import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image
import os
import cv2

import math

def quaternion_to_yaw_pitch_roll(quat):
    """
    Convert quaternion to yaw, pitch, roll (in radians).
    
    Parameters:
    quat: list or tuple of 4 elements [q0, q1, q2, q3]
    
    Returns:
    yaw, pitch, roll: tuple of floats in radians
    """
    q0, q1, q2, q3 = quat

    # Roll (rotation around x-axis)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around y-axis)
    sinp = 2 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (rotation around z-axis)
    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw, pitch, roll


# Function to load data and extract the frame data
def load_data(csv_file, frame_number):
    df = pd.read_csv(csv_file)
    frame_data = df[df['frame_id'] == frame_number].iloc[0]
    return frame_data

# Function to rotate the velocity vector in the XY plane
def rotate_velocity_xy(vx, vy, delta_yaw):
    delta_yaw = -delta_yaw
    cos_yaw = np.cos(delta_yaw)
    sin_yaw = np.sin(delta_yaw)
    vx_rot = vx * cos_yaw - vy * sin_yaw
    vy_rot = vx * sin_yaw + vy * cos_yaw
    return vx_rot, vy_rot

# Function to plot the 3D velocity vector
def plot_velocity_vector(frame_number, csv_file, ax):
    # Load data for the frame
    frame_data = load_data(csv_file, frame_number)
    vx = frame_data['dx']
    vy = frame_data['dy']
    vz = frame_data['dz']
    delta_yaw = frame_data['angle']  # Rotation angle in radians

    # Rotate the velocity vector in the XY plane
    vx_rot, vy_rot = rotate_velocity_xy(vx, vy, delta_yaw)
    
    
    # Plot the velocity vector
    # ax.quiver(0, 0, 0, vx, vy, vz, length=0.05, normalize=True, color='b', label='Original Vector')
    ax.quiver(0, 0, 0, -vx_rot, vy_rot, vz, length=0.05, normalize=True, color='r', label='Rotated Vector')
    # ax.plot([vx, vx_rot], [vy, vy_rot], 0, 'k--', label='Rotation Path')  # Line between original and rotated tips

    ax.set_xlabel('X Velocity')
    ax.set_ylabel('Y Velocity')
    ax.set_zlabel('Z Velocity')
    ax.set_title(f'Frame {frame_number}')
    ax.view_init(elev=90, azim=-180)  # Camera pose

# Function to display the image for the selected frame
def show_image(frame_number, ax_img, images_path):
    # image_path = os.path.join(images_path, f'image_{frame_number:04d}.jpg')
    image_path = os.path.join(images_path, f'image_{frame_number}.jpg')
    img = Image.open(image_path)
    ax_img.imshow(img)
    ax_img.axis('off')

# Main function to create the video
def create_video(csv_file, images_path, output_video_path):
    # Get all image frame numbers from the folder
    frame_numbers = sorted([int(img.split('_')[1].split('.')[0]) for img in os.listdir(images_path) if img.endswith('.jpg')])
    
    # Prepare the video writer
    frame_rate = 30  # Frames per second
    video_size = (1200, 600)  # Output video size (adjust to match your subplots)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_video_path, fourcc, frame_rate, video_size)
    
    for frame_number in frame_numbers:
        # Create a 1x2 grid for subplots
        fig = plt.figure(figsize=(11, 5))
        ax_img = fig.add_subplot(121)  # Image subplot
        ax_3d = fig.add_subplot(122, projection='3d')  # 3D velocity vector subplot
        
        # Show the image and plot the velocity vector
        show_image(frame_number, ax_img, images_path)
        plot_velocity_vector(frame_number, csv_file, ax_3d)
        
        # Save the current figure to a buffer
        plt.tight_layout()
        plt.savefig('temp_frame.png')
        plt.close(fig)
        
        # Read the saved frame and add it to the video
        frame = cv2.imread('temp_frame.png')
        frame = cv2.resize(frame, video_size)  # Ensure correct size
        video_writer.write(frame)
    
    # Release the video writer and clean up
    video_writer.release()
    os.remove('temp_frame.png')
    print(f"Video saved at {output_video_path}")

# Run the script
if __name__ == '__main__':
    folder_data = "2025-02-28 10:38:24.934320"
    folder = "/home/vs/ardupilot_docker/dataset/Reasoning/Letters/" + folder_data
    csv_file = folder + '/data.csv'  # Your CSV file path
    images_path = folder + "/images"
    output_video_path = folder + '/output_video.mp4'
    
    create_video(csv_file, images_path, output_video_path)
