"""Launch one self-contained counting experiment with an immutable scene."""
import json
import os
from pathlib import Path
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ari_counting_sim.scene import generate_scene, write_world
from ari_counting_sim.robot import prepare_robot, prepare_semantic


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    share = Path(get_package_share_directory('ari_counting_sim'))
    description = Path(get_package_share_directory('ari_description'))
    moveit = Path(get_package_share_directory('ari_moveit_config'))
    arm = value('arm')
    if arm not in ['left', 'right']:
        raise ValueError('arm must be left or right')
    scene = generate_scene(int(value('ball_count')), int(value('seed')), yaml.safe_load(Path(value('config')).read_text()))
    folder = Path(value('output_dir'))/'scenes'/uuid.uuid4().hex
    folder.mkdir(parents=True)
    scene.update({'scene_id': folder.name, 'robot_model': 'v2', 'default_arm': arm})
    (folder/'scene.json').write_text(json.dumps(scene, indent=2))
    write_world(scene, share/'worlds/counting.world', folder/'counting.world')
    robot = prepare_robot(description, share/'config/controllers.yaml', scene)
    semantic = prepare_semantic(moveit)
    (folder/'robot.urdf').write_text(robot)
    (folder/'robot.srdf').write_text(semantic)
    (folder/'controllers.yaml').write_text((share/'config/controllers.yaml').read_text())
    simulation = {'use_sim_time': True}
    kin = {f'arm_{s}': {'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                        'kinematics_solver_timeout': 0.1, 'position_only_ik': True}
           for s in ['left', 'right']}
    controllers = {f'arm_{s}_controller': {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                   'default': True, 'joints': [f'arm_{s}_{i}_joint' for i in range(1,6)]} for s in ['left','right']}
    controllers['controller_names'] = ['arm_left_controller', 'arm_right_controller']
    ompl = yaml.safe_load((moveit/'config/ompl_planning.yaml').read_text())
    ompl.update({'planning_plugin': 'ompl_interface/OMPLPlanner',
                 'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                 'start_state_max_bounds_error': 0.1})
    params = {**simulation, 'robot_description': robot, 'robot_description_semantic': semantic,
              'robot_description_kinematics': kin, 'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
              'robot_description_planning': {'joint_limits': {f'arm_{s}_{i}_joint': {
                  'has_velocity_limits': True, 'max_velocity': 0.8,
                  'has_acceleration_limits': True, 'max_acceleration': 0.5}
                  for s in ['left', 'right'] for i in range(1, 6)}},
              'ompl': ompl, 'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
              'moveit_simple_controller_manager': controllers, 'publish_robot_description_semantic': True,
              'trajectory_execution.allowed_start_tolerance': 0.05,
              'publish_planning_scene': True, 'publish_geometry_updates': True,
              'publish_state_updates': True, 'publish_transforms_updates': True}
    robot_node = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[simulation, {'robot_description': robot}], output='screen')
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity','ari','-topic','robot_description','-timeout','120'], output='screen')
    spawners = [Node(package='controller_manager', executable='spawner', arguments=[name, '--controller-manager-timeout', '120'], output='screen')
                for name in ['joint_state_broadcaster','arm_left_controller','arm_right_controller','head_controller']]
    server=ExecuteProcess(cmd=['gzserver','--verbose','--lockstep','-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',str(folder/'counting.world'), '--ros-args', '--param', 'publish_rate:=200.0'], output='screen')
    move_group=Node(package='moveit_ros_move_group', executable='move_group', parameters=[params], output='screen')
    bridge=Node(package='ari_counting_sim', executable='scene_bridge', parameters=[simulation, {'scene_file': str(folder/'scene.json')}], output='screen')
    gateway=Node(package='ari_counting_tools', executable='gateway', parameters=[simulation, {'robot_description': robot, 'arm': arm, 'scene_file': str(folder/'scene.json'), 'output_dir': value('output_dir')}], output='screen')
    def failed_process(event, context):
        if context.is_shutdown: return []
        return [EmitEvent(event=Shutdown(reason='Robot spawning or controller loading failed'))] if event.returncode else []
    def spawned(event, context):
        return failed_process(event, context) if event.returncode else spawners
    def critical_exit(event, context):
        if context.is_shutdown: return []
        return [EmitEvent(event=Shutdown(reason='A required experiment process exited'))]
    handlers=[RegisterEventHandler(OnProcessExit(target_action=process, on_exit=critical_exit))
              for process in [server,move_group,bridge,gateway]]
    handlers += [RegisterEventHandler(OnProcessExit(target_action=process,on_exit=failed_process)) for process in spawners]
    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join([str(description.parent), str(Path(get_package_share_directory('pal_urdf_utils')).parent), os.environ.get('GAZEBO_MODEL_PATH','')])),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        *handlers, server,
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        robot_node,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)), spawn,
        move_group, bridge, gateway,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('ari_counting_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('ball_count', default_value='10'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('arm', default_value='right', choices=['left','right']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(share/'config/experiment.yaml')),
        DeclareLaunchArgument('output_dir', default_value='/data'),
        OpaqueFunction(function=setup),
    ])
