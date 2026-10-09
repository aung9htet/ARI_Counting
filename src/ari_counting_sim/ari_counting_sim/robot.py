"""Adapt expanded PAL descriptions without altering the upstream checkouts."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from .scene import child, vector, camera_sensor


def prepare_robot(description_share, controllers, scene):
    xml = xacro.process_file(str(Path(description_share)/'robots/ari.urdf.xacro'), mappings={
        'robot_model': 'v2', 'end_effector': 'no-end-effector', 'use_sim_time': 'true',
        'gazebo_version': 'classic', 'rgbd_sensors': 'false', 'laser_model': 'no-laser',
    }).toxml()
    root = ET.fromstring(xml)
    for joint in root.findall('joint'):
        if joint.get('name', '').startswith('caster_'):
            joint.set('type', 'fixed')
            for tag in ['axis', 'limit', 'dynamics', 'safety_controller']:
                for element in joint.findall(tag):
                    joint.remove(element)
    for side in ['left', 'right']:
        for index in [2, 4]:
            interface = root.find(f"ros2_control/joint[@name='arm_{side}_{index}_joint']/state_interface[@name='position']")
            child(interface, 'param', '0.1', name='initial_value')
    # Only our RGB cameras are enabled, keeping capture topics and calibration explicit.
    for tag in root.findall('gazebo'):
        for sensor in tag.findall('sensor'):
            tag.remove(sensor)
        for plugin in tag.findall('plugin'):
            if plugin.get('name') == 'gazebo_ros2_control':
                plugin.find('parameters').text = str(controllers)
    child(root, 'link', name='world')
    anchor = child(root, 'joint', name='experiment_base_anchor', type='fixed')
    child(anchor, 'parent', link='world'); child(anchor, 'child', link='base_footprint')
    child(anchor, 'origin', xyz='0 0 0', rpy='0 0 0')
    # Gazebo must retain the world joint instead of reducing it into the base.
    gz = child(root, 'gazebo', reference='experiment_base_anchor')
    child(gz, 'preserveFixedJoint', 'true')
    c = scene['cameras']['robot']
    child(root, 'link', name='robot_camera_link')
    joint = child(root, 'joint', name='robot_camera_mount', type='fixed')
    child(joint, 'parent', link='base_link'); child(joint, 'child', link='robot_camera_link')
    child(joint, 'origin', xyz=vector(c['position']), rpy=vector(c['rpy']))
    child(root, 'link', name=c['frame'])
    optical = child(root, 'joint', name='robot_camera_optical_joint', type='fixed')
    child(optical, 'parent', link='robot_camera_link'); child(optical, 'child', link=c['frame'])
    child(optical, 'origin', xyz='0 0 0', rpy=vector([-math.pi/2,0,-math.pi/2]))
    camera_sensor(child(root, 'gazebo', reference='robot_camera_link'), 'robot', c)
    return ET.tostring(root, encoding='unicode')


def prepare_semantic(moveit_share):
    root = ET.parse(Path(moveit_share)/'config/ari_v2.srdf').getroot()
    for side in ['left', 'right']:
        group = root.find(f"group[@name='arm_{side}']")
        group.clear(); group.set('name', f'arm_{side}')
        child(group, 'chain', base_link='base_link', tip_link=f'arm_{side}_hand_link')
    return ET.tostring(root, encoding='unicode')


def joint_limits(robot_xml, side):
    root = ET.fromstring(robot_xml)
    values = []
    for joint in root.findall('joint'):
        if joint.get('type') == 'revolute' and joint.get('name', '').startswith(f'arm_{side}_'):
            limit = joint.find('limit')
            safety = joint.find('safety_controller')
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            if safety is not None:
                lower = max(lower, float(safety.get('soft_lower_limit', lower)))
                upper = min(upper, float(safety.get('soft_upper_limit', upper)))
            values.append({'name': joint.get('name'), 'lower': lower, 'upper': upper,
                           'velocity': float(limit.get('velocity')), 'axis': joint.find('axis').get('xyz')})
    return sorted(values, key=lambda j: j['name'])
