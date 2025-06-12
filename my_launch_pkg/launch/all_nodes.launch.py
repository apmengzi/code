import launch
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定义包路径
    lslidar_share = FindPackageShare('lslidar_driver')
    tbot_share = FindPackageShare('tbot_description')
    nav_pkg_share = FindPackageShare('fishbot_navigation2')
    usb_cam_share = FindPackageShare('usb_cam')
    apriltag_share = FindPackageShare('apriltag_ros')
    # slam_toolbox_share = FindPackageShare('slam_toolbox')
    
    # 定义launch文件
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([lslidar_share, 'launch', 'lsn10p_launch.py'])]),
    )
    
    tbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([tbot_share, 'launch', 'display.launch'])]),
    )
    
    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution([slam_toolbox_share, 'launch', 'online_async_launch.py'])]),
    #     launch_arguments={'use_sim_time': 'True'}.items(),
    # )
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([nav_pkg_share, 'launch', 'navigation2.launch.py'])]),
    )
    
    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([usb_cam_share, 'launch', 'camera.launch.py'])]),
    )
    
    # 定义节点
    my_base_node = Node(
        package='my_base',
        executable='my_base',
    )
    
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        parameters=[{'port': 8080, 'address': '0.0.0.0'}],
    )
    
    apriltag_params_file = PathJoinSubstitution([apriltag_share, 'cfg', 'tags_36h11.yaml'])
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[('image_rect', '/camera1/image_raw'), ('camera_info', '/camera1/camera_info')],
        parameters=[apriltag_params_file],
    )
    
    nav2_custom_node = Node(
        package='nav2_custom',
        executable='nav2_custom_node',
    )
    
    tag_tf_node = Node(
        package='tag_tf_and_position_printer',
        executable='tag_tf_and_position_printer_node',
    )
    
    # 定义动作列表，设置递增延迟
    actions = [
        (my_base_node, 0.0),
        (lslidar_launch, 0.0),
        (tbot_launch, 0.0),
        #(slam_toolbox_launch, 1.0),
        (usb_cam_launch, 1.0),
        (apriltag_node, 0.0),
        (web_video_server_node, 1.0),
        (nav_launch, 1.0),
        (tag_tf_node, 3.0),
        (nav2_custom_node, 0.0),
    ]
    
    # 创建延迟动作
    delayed_actions = [
        TimerAction(period=delay, actions=[action]) for action, delay in actions
    ]
    
    # 返回launch描述
    return launch.LaunchDescription(delayed_actions)