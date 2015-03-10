#description URDF
roslaunch fsr_backpack_description/launch/description_backpack.launch

#modulos do MD Schiebel + gy80
roslaunch fsr_backpack_arm_node/launch/fsr_backpack_arm_node_imu.launch

#odometria + xsens
roslaunch fsr_backpack_odometry/launch/fsr_backpack_odometry.launch

#data-viewer
roslaunch fsr_backpack_data_viewer/launch/fsr_backpack_simple_data_viewer.launch


