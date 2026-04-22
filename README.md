# Comandos ROS2
- Mover cubo por terminal `ros2 topic pub --once /cube_command std_msgs/msg/String "{data: 'UM'}"`
- Iniciar estado por terminal `ros2 topic pub --once /cube_state_raw std_msgs/msg/String "{data: 'BUUBUULUUBBBRRRRRRURRUFFUFFRDDFDDFDDFLLFLLFLLLLDBBDBBD'}"`
- Solicitar estado reorientado (solo para pruebas) por terminal `ros2 service call /get_cube_state std_srvs/srv/Trigger {}`