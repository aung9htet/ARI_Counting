import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from ari_counting_sim.scene import generate_scene, write_world, look_at

ROOT=Path(__file__).resolve().parents[1]
CONFIG=yaml.safe_load((ROOT/'src/ari_counting_sim/config/experiment.yaml').read_text())


@pytest.mark.parametrize('count',[1,25,100])
def test_scene_geometry(count,tmp_path):
    scene=generate_scene(count,42,CONFIG)
    assert len(scene['objects'])==count
    assert sum(scene['counts_by_color'].values())==count
    assert scene==generate_scene(count,42,CONFIG)
    t=CONFIG['table']; b=CONFIG['balls']
    for i,obj in enumerate(scene['objects']):
        x,y,z=obj['position']
        assert abs(x-t['center_x'])<=t['length']/2-b['radius']-b['clearance']
        assert abs(y-t['center_y'])<=t['width']/2-b['radius']-b['clearance']
        assert z==t['height']+b['radius']
        assert obj['color'] in b['palette']
        for other in scene['objects'][:i]:
            assert math.dist(obj['position'],other['position'])>=2*b['radius']+b['clearance']
    world=tmp_path/'scene.world'
    write_world(scene,ROOT/'src/ari_counting_sim/worlds/counting.world',world)
    tree=ET.parse(world)
    assert len(tree.findall('.//sensor'))==2
    assert len(tree.findall('.//model'))==count+8


@pytest.mark.parametrize('value',[0,101,-1,True,1.5,'10'])
def test_invalid_counts(value):
    with pytest.raises(ValueError): generate_scene(value,0,CONFIG)


def test_seed_changes_layout():
    assert generate_scene(10,1,CONFIG)['objects']!=generate_scene(10,2,CONFIG)['objects']


def test_camera_pitch():
    assert look_at([0,0,3],[0,0,0])[1]==math.pi/2
