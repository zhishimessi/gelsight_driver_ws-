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
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        parameters=[
            "/home/donghy/Desktop/thesis/gelsight_driver/Bnz/ROS2/gelsight_driver_ws/src/calibration/config/camera_params.yaml"
        ],
        output="screen"
    )

    depth_realtime_node = Node(
            package='calibration',  
            executable='depth_realtime',  
            name='depth_realtime_node',  
            output='screen'  
    )

    gelsight_driver_node = Node(
        package="markermotion",
        executable="gelsight_driver",
        name="gelsight_driver",
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
        gelsight_driver_node,
        rviz2_node
    ])
