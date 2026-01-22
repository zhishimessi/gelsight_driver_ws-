from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  
from launch.substitutions import PathJoinSubstitution  

def generate_launch_description():
    markermotion_pkg = FindPackageShare(package="markermotion").find("markermotion")
    rviz_config_path = PathJoinSubstitution(
        [markermotion_pkg, "config", "testzz.rviz"] 
    )

    image_publisher_node = Node(
        package="image_publisher",
        executable="image_publisher_node",
        name="image_publisher",
        arguments=[
            "--ros-args",
            "-p", "filename:=/home/donghy/Desktop/thesis/gelsight_driver/Bnz/ROS2/gelsight_driver_ws/src/markermotion/video/video.avi"
        ],
        output="screen"
    )

    depth_realtime_node = Node(
            package='calibration',  
            executable='depth_realtime',  
            name='depth_realtime_node',  
            output='screen'  
    )

    marker_ros2_node = Node(
        package="markermotion",
        executable="marker_ros2",
        name="marker_ros2",
        output="screen"
    )

    marker_cv2_node = Node(
        package="markermotion",
        executable="marker_cv2",
        name="marker_cv2",
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],  
        output="screen"
    )

    return LaunchDescription([
        image_publisher_node,
        depth_realtime_node,
        marker_ros2_node,
        # marker_cv2_node,
        rviz2_node
    ])
