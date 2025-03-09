from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():

    ns = LaunchConfiguration('ns', default='/uav1')
    
    grid_map_node = Node(
        package='grid_map2',
        executable='grid_map_node',
        name='grid_map_node',
        namespace=ns,
        remappings=[
            ('grid_map/odom', '/uav1/mavros/local_position/odom'),
            ('grid_map/cloud', '/map_generator/obstacles'),
            ('grid_map/pose', 'nouse1'),
            ('grid_map/depth', 'camera/depth/image_rect_raw')
        ],
        output='screen',
        parameters=[
            {'grid_map/resolution': 0.15},
            {'grid_map/map_size_x': 100.0},
            {'grid_map/map_size_y': 100.0},
            {'grid_map/map_size_z': 20.0},
            {'grid_map/local_update_range_x': 10.0},
            {'grid_map/local_update_range_y': 10.0},
            {'grid_map/local_update_range_z': 5.5},
            {'grid_map/obstacles_inflation': 0.099},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': 0.0},
            {'grid_map/cx': 320.0},
            {'grid_map/cy': 240.0},
            {'grid_map/fx': 184.75},
            {'grid_map/fy': 415.69},
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 6},
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.3},
            {'grid_map/max_ray_length': 5.0},
            {'grid_map/visualization_truncate_height': 20.0},
            {'grid_map/show_occ_time': True},
            {'grid_map/pose_type': 2},
            {'grid_map/esdf_slice_height': 0.80},
            {'grid_map/show_esdf_time': False},
            {'grid_map/local_bound_inflate': 1.0},
            {'grid_map/frame_id': 'map'}
        ]
    )

    return LaunchDescription([grid_map_node])
