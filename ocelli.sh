#April 29, 2023
#
#this is meant to run all necessary bash commands to operate Ocelli
#this is running on multiple tabs to allow troubleshooting.
#RPI IS NOT AUTOMATED YET. PLEASE LOG IN THE RPI TO START THE LEPTON's SCRIPT.

echo "starting script..."

#setup roscore
gnome-terminal --tab -- bash -c 'source ~/catkin_ws/devel/setup.bash && roscore'
sleep 10
#setup lepton/RPi
sudo modprobe v4l2loopback video_nr=3 card_label="lepton_r" exclusive_caps=1 max_buffers=3
gnome-terminal --tab -- bash -c 'source ~/catkin_ws/devel/setup.bash && roslaunch gscam_streamer gscam_config.launch'
sleep 10
#setup KinectV2
gnome-terminal --tab -- bash -c 'source ~/catkin_ws/devel/setup.bash && roslaunch kinect2_bridge kinect2_bridge.launch'
sleep 10
#setup TF
gnome-terminal --tab -- bash -c 'source ~/catkin_ws/devel/setup.bash && rosrun tf static_transform_publisher 0 0 0 0 0 0 kinect2_link base_link 30 && read'
sleep 10
#setup RegisterCam code
gnome-terminal --tab -- bash -c 'source ~/catkin_ws/devel/setup.bash && rosrun registered_cameras registered_cameras_node.py _calibration_yaml:=~/Ocelli_Param.yaml && read'
sleep 10
#start Rviz
gnome-terminal --tab -- bash -c 'source ~/catkin_ws/devel/setup.bash && rosrun rviz rviz'
sleep 5
echo "end of script"
