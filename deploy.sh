#!/bin/bash
echo "Starting setup on TurtleBot..."

# Update and install necessary packages
echo "Updating system and installing dependencies..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip ros-humble-geometry-msgs

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Copy code to TurtleBot workspace
echo "Deploying code..."
mkdir -p ~/robot_workspace
cp -r controller.py ~/robot_workspace/

# Test ROS setup
echo "Testing ROS setup..."
source /opt/ros/humble/setup.bash
echo "ROS environment initialized."

echo "Setup complete! Run the controller with: python3 ~/robot_workspace/controller.py"
