from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # npc_with_quest_node = Node(
    #     package="route_planner",
    #     executable="npc_with_quest"
    # )

    # path_planner_node = Node(
    #     package="route_planner",
    #     executable="path_planner"
    # )

    # user_node = Node(
    #     package="route_planner",
    #     executable="user"
    # )
    
    map_dir = "~/ws_ros2/src/route_planner/test_map/test.yaml"
    
    map_node = Node(
        package="nav2_map_server",
        node_executable="map_server",
        node_name="map_server",
        output="screen",
        parameters=[{'yaml_filename': map_dir}]
    )

    # ld.add_action(npc_with_quest_node)
    # ld.add_action(path_planner_node)
    ld.add_action(map_node)
    return ld