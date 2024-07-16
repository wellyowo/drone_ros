#!/bin/bash

# Base URL of the FTP server
FTP_BASE_URL="ftp://arg.lab.nycu.edu.tw/arg-projectfile-download/robotx-2022/"

# Ask for the relative path of the folder on the FTP server
read -p "Enter the relative folder path (e.g., 3D_env_meshs/sanxiantai_bridge/meshes/): " ftp_folder_path

# Default local directory for downloads
local_dir_prefix="../catkin_ws/src/vrx/vrx_gazebo/models"
read -p "Enter the local download directory (e.g., sanxiantai_bridge/meshes): " local_dir
local_dir="${local_dir_prefix}/${local_dir}"
# Ensure the local directory exists
mkdir -p "$local_dir"

# Full URL of the folder
full_url="${FTP_BASE_URL}${ftp_folder_path}"

# Download the contents of the folder directly into the specified local directory
wget -r -nd -np -P "$local_dir" "$full_url"

echo "Downloaded contents of $ftp_folder_path to $local_dir"
