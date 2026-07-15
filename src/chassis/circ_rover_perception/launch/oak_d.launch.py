#!/usr/bin/env python3
# =====================================================================
# OAK-D Lite bring-up  (circ2025)  -- TODO en un solo archivo
#   1) Driver depthai (parametros inline, sin yaml aparte).
#      Publica su propio TF: base_link -> oak -> frames opticos.
#   2) depthimage_to_laserscan: profundidad -> /scan anclado al TF.
#
# El bloque MONTAJE (i_tf_cam_pos_*) es lo unico que ajustas.
# =====================================================================
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    # --- 1) Camara OAK-D Lite (driver depthai, params inline) -------
    oak_component = ComposableNode(
        package='depthai_ros_driver',
        plugin='depthai_ros_driver::Camera',
        name='oak',
        parameters=[{
            # --- como transmite ---
            'camera.i_pipeline_type': 'RGBD',     # RGB + depth. 'Stereo' si no quieres RGB.
            'camera.i_nn_type': 'none',
            'camera.i_enable_imu': False,         # la LITE NO trae IMU -> debe ir False
            'camera.i_usb_speed': 'SUPER_PLUS',   # baja a 'HIGH' si el USB se inestabiliza

            # --- TF / MONTAJE respecto a base_link (AJUSTAR) ---
            'camera.i_publish_tf_from_calibration': True,  # publica el arbol de frames
            'camera.i_tf_camera_model': 'OAK-D-LITE',
            'camera.i_tf_base_frame': 'oak',
            'camera.i_tf_parent_frame': 'base_link',
            'camera.i_tf_cam_pos_x': 0.20,        # AJUSTAR: adelante (m)
            'camera.i_tf_cam_pos_y': 0.0,         # AJUSTAR: lateral (m, +=izq)
            'camera.i_tf_cam_pos_z': 0.15,        # AJUSTAR: altura (m)
            'camera.i_tf_cam_roll': 0.0,
            'camera.i_tf_cam_pitch': 0.0,         # AJUSTAR: +inclina abajo. 0.26 rad ~= 15 grados
            'camera.i_tf_cam_yaw': 0.0,

            # --- streams ---
            'rgb.i_resolution': '1080P',
            'rgb.i_fps': 15.0,
            'stereo.i_resolution': '400P',        # mono de la LITE: '400P' o '480P'
            'stereo.i_fps': 15.0,
            'stereo.i_align_depth': False,        # depth queda en oak_right_camera_optical_frame
            'stereo.i_lr_check': True,
            'stereo.i_subpixel': False,
            'stereo.i_stereo_conf_threshold': 5,  # sube (10-15) para menos ruido
        }],
    )

    oak_container = ComposableNodeContainer(
        name='oak_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[oak_component],
        output='screen',
    )

    # --- 2) Profundidad -> LaserScan --------------------------------
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.4,      # bajo este valor el depth de la LITE es poco fiable
            'range_max': 8.0,
            'scan_height': 10,
            'output_frame': 'oak_right_camera_optical_frame',  # VERIFICAR con camera_info
        }],
        remappings=[
            ('depth', '/oak/stereo/image_raw'),
            ('depth_camera_info', '/oak/stereo/camera_info'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([oak_container, depth_to_scan])