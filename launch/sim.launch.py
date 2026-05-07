import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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
        # 4. El Sim Controller (El que gotea los comandos cada 0.5s)
        Node(
            package='rubik_viz',
            executable='rubik_sim_controller',
            name='sim_controller',
            output='screen'
        ),
        # Extra: RViz2 configurado
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            condition=None
        )
    ])