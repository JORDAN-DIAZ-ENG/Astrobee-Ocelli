## Astrobee-Ocelli
test

## Overview
Ocelli is a software prototype designed to integrate multiple data streams from the NASA Astrobee system. It combines thermal imaging from the Lepton 3.5 thermal camera with depth imaging from a Kinect v2, resulting in a live 3D heatmap. This software facilitates the fusion of data streams of different resolutions to provide valuable insights for various applications.

## Features
- Integration of thermal imaging from Lepton 3.5 thermal camera.
- Fusion of depth imaging from Kinect v2.
- Real-time stitching of data streams to create a live 3D heatmap.
- Calibration of cameras for accurate data alignment.

## System Architecture
Ocelli operates in the following environment:
- Lepton 3.5 thermal camera connected to a Raspberry Pi.
- Kinect v2 depth sensor.
- Linux machine running ROS Melodic.
- Utilizes Video4Linux2 to transmit thermal imaging data from Lepton 3.5 to the ROS environment.
- Python script for stitching together the thermal and depth streams in ROS.

## Calibration
Before running Ocelli, it's crucial to calibrate the Lepton 3.5 thermal camera and Kinect v2 depth sensor for accurate data alignment. This is achieved by capturing images of a checkerboard pattern and generating a YAML file containing the calibration parameters.

## Usage
1. Connect the Lepton 3.5 thermal camera to the Raspberry Pi and ensure it's streaming data.
2. Connect the Kinect v2 depth sensor to the ROS environment.
3. Launch the Ocelli software on the ROS environment.
4. Provide the necessary calibration parameters and start the stitching process.
5. Visualize the live 3D heatmap generated by Ocelli.

## Example
![Final Product](path_to_image)

## Acknowledgments
- Acknowledge any third-party libraries or tools used in the development of Ocelli.
- Thank you NASA AMES for the opportunity to work on this project and utilize your Astrobee system.