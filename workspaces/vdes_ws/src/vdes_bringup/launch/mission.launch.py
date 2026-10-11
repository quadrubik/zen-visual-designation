"""Launch file de la mission démo : une node, deux fichiers de configuration du repo.

`config/demo.yaml` et `config/sites/<site>.yaml` sont tous deux au format paramètres ROS 2 :
ils se passent tels quels à la node, sans lecture YAML ici. Les clés imbriquées du site
(`target: {north_m: ...}`) arrivent à la node sous les noms `target.north_m`, `target.east_m`
et `altitude_agl`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONFIG_DIR = '/aeac/config'   # le repo est toujours monté à /aeac dans les conteneurs


def generate_launch_description():
    ld = LaunchDescription()

    sim = DeclareLaunchArgument('sim', default_value='false',
                                description='Vrai en simulation : aucun appel matériel')
    site = DeclareLaunchArgument('site', default_value='sim',
                                 description='Terrain, donc config/sites/<site>.yaml')

    target_node = Node(
        package="vdes_nodes",
        executable="py_target_generation",
        name="target_generator",
        parameters=[f'{CONFIG_DIR}/vdes.yaml'],
    )

    gimbal_node = Node(
            package="vdes_nodes",
            executable="py_gimbal_simulation",
            name="gimbal_simulation",
            parameters=[f'{CONFIG_DIR}/vdes.yaml'],
    )

    camera_node = Node(
            package="vdes_nodes",
            executable="py_camera_simulation",
            name="camera_simulation",
            parameters=[f'{CONFIG_DIR}/vdes.yaml'],
    )

    ld.add_action(target_node)
    ld.add_action(gimbal_node)
    ld.add_action(camera_node)

    ld.add_action(sim)
    ld.add_action(site)
    return ld
