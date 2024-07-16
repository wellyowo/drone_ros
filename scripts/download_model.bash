#!/bin/bash

# Base URL of the FTP server
FTP_BASE_URL="ftp://arg.lab.nycu.edu.tw/arg-projectfile-download/robotx-2022/"

# Ask for the relative path of the file on the FTP server
read -p "Enter the relative file path (e.g., acme_model/cave.zip): " ftp_file_path

# Ask for the local download directory
read -p "Enter the local download directory (e.g., logs): " local_dir

# Create the local directory if it doesn't exist
mkdir -p "$local_dir"

# Full URL of the file
full_url="${FTP_BASE_URL}${ftp_file_path}"

# Download the file
wget -P "$local_dir" "$full_url"

echo "Downloaded $ftp_file_path to $local_dir"
