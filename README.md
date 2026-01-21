 ```bash 
 
 0.colcon build --symlink-install
# calibration
 1.run img_processor.py to get image for cali (remember -i $num of pictures$)
 
 2.run calibration.py 
 
     -use "w, s, a, d" to adjust the circle position, use "m,n" to adjust the radius of the circle
     -press "Esc" after the adjustment 

# normals, depth, marker motion visualization
 1.video mode

    ros2 launch markermotion testzz.launch.py
    
 2.realtime

    ros2 launch markermotion gelsight_driver.launch.py

 
