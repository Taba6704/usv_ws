from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    humidity_node = Node(
        package='humidity_sensor',
        executable='humidity_sensor_node',
        name='humidity_sensor_node',
        output='screen'
    )

    lcd_node = Node(
        package='humidity_sensor',
        executable='lcd_display_node',
        name='lcd_display_node',
        output='screen'
    )

    lcd_node_delayed = TimerAction(
        period = 2.0,
        actions = [lcd_node]
    )

    rc_node = Node(
        package='rc_controller',
        executable='rc_controller_node',
        name='rc_controller_node',
        output='screen'
    )

    thruster_node = Node(
        package='usv_thrusters',
        executable='thruster_driver',
        name='thruster_driver',
        output='screen'
    )

    cmd_vel_mux_node = Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        output='screen'
    )

    return LaunchDescription([
        humidity_node,
        lcd_node_delayed,
        rc_node,
        thruster_node,
        cmd_vel_mux_node,
    ])