from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ursim_node = Node(
        package='ur_client_library',
        executable='start_ursim.sh',
        arguments=['ur5e'],
        output='screen'
    )

    pkg_ur_driver = FindPackageShare('ur_robot_driver')

    ur_driver_launch = PathJoinSubstitution([pkg_ur_driver, 'launch', 'ur_control.launch.py'])
    ur_driver_include = IncludeLaunchDescription(PythonLaunchDescriptionSource([ur_driver_launch]), 
                                                launch_arguments={'ur_type': 'ur5e', 'robot_ip': '192.168.56.101', 'launch_rviz': 'true'}.items())

    usb_cam_node = Node(package='usb_cam',
                        executable='usb_cam_node_exe')

    camera_node = Node(package='camera_subscriber',
                       executable='camera_subscriber')

    control_node = Node(package='control_publisher',
                        executable='control_publisher')

    power_on_srv = ExecuteProcess(cmd=[[FindExecutable(name="ros2"),
                                        " service call ",
                                        "/dashboard_client/power_on ",
                                        "std_srvs/srv/Trigger "]], shell=True)

    brake_release_srv = ExecuteProcess(cmd=[[FindExecutable(name="ros2"),
                                        " service call ",
                                        "/dashboard_client/brake_release ",
                                        "std_srvs/srv/Trigger "]], shell=True)
    
    load_program_srv = ExecuteProcess(cmd=[[FindExecutable(name="ros2"),
                                        " service call ",
                                        "/dashboard_client/load_program ",
                                        "ur_dashboard_msgs/srv/Load ",
                                        r'"{filename: "ext_ctrl.urp"}"']], shell=True)
    
    play_srv = ExecuteProcess(cmd=[[FindExecutable(name="ros2"),
                                        " service call ",
                                        "/dashboard_client/play ",
                                        "std_srvs/srv/Trigger "]], shell=True)
    
    delayed_start = TimerAction(period=45.0, actions=[brake_release_srv, play_srv])
    
    delayed_ur_driver = TimerAction(period=30.0, actions=[ur_driver_include, power_on_srv,
                                                        load_program_srv])
    
    delayed_start = TimerAction(period=45.0, actions=[brake_release_srv, play_srv])
    
    return LaunchDescription([ursim_node,
                              usb_cam_node,
                              camera_node,
                              control_node,
                              delayed_ur_driver,
                              delayed_start
                              ])