"""Deterministic scene generation, independent of ROS and Gazebo imports."""
import argparse
import copy
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET
import yaml


def look_at(position, target):
    dx, dy, dz = [b - a for a, b in zip(position, target)]
    if dx == dy == dz == 0:
        raise ValueError('camera position and target must differ')
    return [0.0, -math.atan2(dz, math.hypot(dx, dy)), math.atan2(dy, dx)]


def generate_scene(count, seed, config):
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 100:
        raise ValueError('ball_count must be an integer from 1 to 100')
    cfg = copy.deepcopy(config)
    t, b, camera = cfg['table'], cfg['balls'], cfg['camera']
    for value in [t['length'], t['width'], t['height'], t['thickness'], b['radius']]:
        if not math.isfinite(value) or value <= 0:
            raise ValueError('geometry dimensions must be positive and finite')
    if not 0 < t['thickness'] < t['height'] or b['clearance'] < 0:
        raise ValueError('invalid thickness or clearance')
    margin = b['radius'] + b['clearance']
    if min(t['length'], t['width']) <= 2 * margin:
        raise ValueError('table is too small for the configured balls')
    palette = b['palette']
    if not palette or any(len(rgba) != 4 or any(not math.isfinite(x) or not 0 <= x <= 1 for x in rgba) for rgba in palette.values()):
        raise ValueError('palette must contain names with RGBA values in [0,1]')
    if any(not isinstance(camera[k], int) or camera[k] <= 0 for k in ['width', 'height', 'fps']) or not 0 < camera['horizontal_fov'] < math.pi:
        raise ValueError('invalid camera dimensions, FPS or field of view')
    rng = random.Random(seed)
    objects = []
    minimum = 2 * b['radius'] + b['clearance']
    for _ in range(100000):
        if len(objects) == count:
            break
        x = rng.uniform(t['center_x'] - t['length']/2 + margin, t['center_x'] + t['length']/2 - margin)
        y = rng.uniform(t['center_y'] - t['width']/2 + margin, t['center_y'] + t['width']/2 - margin)
        if any(math.hypot(x - obj['position'][0], y - obj['position'][1]) < minimum for obj in objects):
            continue
        color = rng.choice(list(palette))
        objects.append({'id': f'ball_{len(objects):03d}', 'color': color,
                        'rgba': palette[color], 'radius': b['radius'],
                        'position': [x, y, t['height'] + b['radius']]})
    if len(objects) != count:
        raise ValueError('unable to place requested balls without overlap; enlarge table or reduce radius')
    cameras = {}
    for name in ['overhead', 'front', 'robot']:
        c = copy.deepcopy(camera[name])
        if any(len(c[k]) != 3 or any(not math.isfinite(v) for v in c[k]) for k in ['position', 'target']):
            raise ValueError('camera poses must contain three finite coordinates')
        c['rpy'] = look_at(c['position'], c['target'])
        c.update({key: camera[key] for key in ['width', 'height', 'fps', 'horizontal_fov']})
        c['frame'] = f'{name}_camera_optical_frame'
        c['topic'] = f'/experiment/cameras/{name}'
        cameras[name] = c
    boxes = [{'id': 'table_top', 'size': [t['length'], t['width'], t['thickness']],
              'position': [t['center_x'], t['center_y'], t['height'] - t['thickness']/2]}]
    for i, (sx, sy) in enumerate([(-1,-1), (-1,1), (1,-1), (1,1)]):
        boxes.append({'id': f'table_leg_{i}', 'size': [0.06, 0.06, t['height']-t['thickness']],
                      'position': [t['center_x'] + sx*(t['length']/2-0.06), t['center_y'] + sy*(t['width']/2-0.06), (t['height']-t['thickness'])/2]})
    return {'schema_version': 1, 'seed': seed, 'ball_count': count, 'config': cfg,
            'objects': objects, 'table_boxes': boxes, 'cameras': cameras,
            'counts_by_color': {c: sum(obj['color']==c for obj in objects) for c in palette}}


def child(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def vector(values):
    return ' '.join(str(v) for v in values)


def camera_sensor(parent, name, camera):
    sensor = child(parent, 'sensor', name=name, type='camera')
    child(sensor, 'always_on', 'true')
    child(sensor, 'update_rate', camera['fps'])
    child(sensor, 'visualize', 'false')
    c = child(sensor, 'camera', name=name)
    child(c, 'horizontal_fov', camera['horizontal_fov'])
    im = child(c, 'image')
    child(im, 'width', camera['width']); child(im, 'height', camera['height']); child(im, 'format', 'R8G8B8')
    clip = child(c, 'clip'); child(clip, 'near', 0.02); child(clip, 'far', 20)
    plugin = child(sensor, 'plugin', name=f'{name}_ros_camera', filename='libgazebo_ros_camera.so')
    ros = child(plugin, 'ros'); child(ros, 'namespace', camera['topic'])
    child(ros, 'remapping', f'{name}/image_raw:=image_raw')
    child(ros, 'remapping', f'{name}/camera_info:=camera_info')
    child(plugin, 'camera_name', name)
    child(plugin, 'frame_name', camera['frame'])


def write_world(scene, template, destination):
    tree = ET.parse(template)
    world = tree.getroot().find('world')
    for obj in scene['table_boxes'] + scene['objects']:
        model = child(world, 'model', name=obj['id']); child(model, 'static', 'true')
        child(model, 'pose', vector(obj['position'] + [0,0,0]))
        link = child(model, 'link', name='link')
        rgba = obj.get('rgba', [0.55,0.35,0.17,1])
        for tag in ['visual', 'collision']:
            element = child(link, tag, name=tag)
            geom = child(element, 'geometry')
            if 'radius' in obj:
                child(child(geom, 'sphere'), 'radius', obj['radius'])
            else:
                child(child(geom, 'box'), 'size', vector(obj['size']))
            if tag == 'visual':
                mat = child(element, 'material'); child(mat, 'ambient', vector(rgba)); child(mat, 'diffuse', vector(rgba))
    for name in ['overhead', 'front']:
        c = scene['cameras'][name]
        model = child(world, 'model', name=f'{name}_camera'); child(model, 'static', 'true')
        child(model, 'pose', vector(c['position'] + c['rpy']))
        camera_sensor(child(model, 'link', name='link'), name, c)
    ET.indent(tree)
    tree.write(destination, encoding='unicode', xml_declaration=True)


def main():
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('ari_counting_sim'))
    p = argparse.ArgumentParser()
    p.add_argument('--ball-count', type=int, default=10); p.add_argument('--seed', type=int, default=0)
    p.add_argument('--config', type=Path, default=share/'config/experiment.yaml')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    scene = generate_scene(args.ball_count, args.seed, yaml.safe_load(args.config.read_text()))
    args.output.mkdir(parents=True, exist_ok=True)
    write_world(scene, share/'worlds/counting.world', args.output/'counting.world')
    (args.output/'scene.json').write_text(json.dumps(scene, indent=2))
