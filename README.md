 0.colcon build --symlink-install
# heightmap reconstruction
 1.ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/donghy/Desktop/thesis/gelsight_driver/Bnz/ROS2/gelsight_driver_ws/src/calibration/config/camera_params.yaml

 2.run img_processor.py to get image for cali (remember -i $num of pictures$)
 
 3.run calibration.py 
 
     -use "w, s, a, d" to adjust the circle position, use "m,n" to adjust the radius of the circle
     -press "Esc" after the adjustment 
 
 4.run test_poission.py

# marker motion visualization
 1.ros2 launch markemotion testzz.launch.py

    display marker motion in video
    
 2.ros2 launch gelsight_driver.launch.py

    display marker motion in realtime
 
