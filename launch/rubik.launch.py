import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def get_real_camera_path():
    """
    Resuelve el enlace simbólico del ID único de la Logitech
    """
    id_path = '/dev/v4l/by-id/usb-046d_Logitech_StreamCam_E90E6626-video-index0'
    if os.path.exists(id_path):
        # os.path.real_path convierte el enlace simbólico en la ruta física real de Linux
        return os.path.realpath(id_path)
    else:
        # Fallback por si acaso algún día se desconecta
        return '/dev/video2'

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('rubik_viz'),
        'rviz',
        'rubik.rviz')
    return LaunchDescription([
        # 1. El Simulador (Visualización y lógica 3D)
        Node(
            package='rubik_viz',
            executable='rubik_sim_node',
            name='sim_node',
            output='screen'
        ),
        # 2. El Solver (Lógica de Kociemba)
        Node(
            package='rubik_viz',
            executable='rubik_solver_node',
            name='solver_node',
            output='screen'
        ),
        # 3. El Sequencer (Traductor de movimientos a X, Y, UM)
        Node(
            package='rubik_viz',
            executable='cube_sequencer',
            name='sequencer_node',
            output='screen'
        ),
        # 4. Nodo de vision (Detecta el estado del cubo y lo publica)
        Node(
            package='rubik_viz',
            executable='cube_vision_node',
            name='vision_node',
            output='screen'
        ),
        # 5. Nodo de camara externo (paquete de ros)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                'video_device': get_real_camera_path(),
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera_link',
                'topic_namespace': 'camera',
                'topic_name': 'image_raw',
                # ==========================================
                # PARÁMETROS DE ILUMINACIÓN Y CÁMARA:
                # ==========================================
                # 1 = Modo Manual, 3 = Modo Automático. 
                # Ponlo en 1 para que te obedezca los valores de abajo.
                'exposure_auto': 1, 
                
                # Tiempo de exposición (Prueba entre 100 y 500 según tu luz)
                'exposure': 250, 
                
                # Brillo general (Suele ir de 0 a 255, prueba un valor alto como 150-180)
                'brightness': 160,
                
                # Ganancia digital (Prueba valores bajos como 30-80 para evitar ruido)
                'gain': 40,
            }]
        ),
        # 6 ur_controller (Ejecuta los movimientos en el robot)
        Node(
            package='rubik_viz',
            executable='ur_cube_controller',
            # Forzamos a que conserve su nombre nativo de C++ por si acaso
            name='ur_cube_controller', 
            output='screen',
            # ESTAS DOS LÍNEAS REPLICAN EL ENTORNO DE ROS2 RUN:
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info']
        ),
        # Extra: RViz2 configurado
        Node(
            package='rviz2',
            executable='rviz2',
            name='rubik_cube_rviz',
            arguments=['-d', rviz_config_dir],
            condition=None
        )
    ])