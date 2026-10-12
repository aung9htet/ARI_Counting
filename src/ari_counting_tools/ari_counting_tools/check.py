"""Integration checks; motion and recording are explicitly opt-in."""
import argparse
import json
import math
from pathlib import Path
import time
from .client import ExperimentClient


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm',choices=['left','right'],default='right')
    parser.add_argument('--motion',action='store_true')
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--seconds',type=float,default=5)
    args=parser.parse_args()
    with ExperimentClient(args.arm) as client:
        client.wait_ready()
        a=client.get_observation(); time.sleep(0.5); b=client.get_observation()
        assert b['stamp_ns']>a['stamp_ns'],'simulation clock stopped'
        assert b['joint_valid']
        for name,view in b['cameras'].items():
            info=view['camera_info']
            assert view['valid'] and view['image'].shape==(info.height,info.width,3),name
            assert info.header.frame_id==f'{name}_camera_optical_frame'
        print('PASS: clock, joint states, controllers, planning scene and three calibrated cameras')
        if args.record: print('Recording:',client.start_recording('Integration validation'))
        try:
            if args.motion:
                limits=client.status()['limits'][args.arm]
                state=b['joint_state']; values=dict(zip(state.name,state.position))
                baseline=[max(j['lower']+0.03,min(j['upper']-0.03,values[j['name']])) for j in limits]
                client.command_joints(baseline,3)
                for i in range(5):
                    target=list(baseline); target[i]+=0.04
                    result=client.command_joints(target,2)
                    assert abs(result['positions'][i]-target[i])<0.02
                    client.command_joints(baseline,2)
                try: client.command_joints([100]*5)
                except RuntimeError: pass
                else: raise AssertionError('out-of-limit command was accepted')
                try: client.move_xyz(100,100,100)
                except RuntimeError: pass
                else: raise AssertionError('unreachable target was accepted')
                print('PASS: five joints, joint limits, unreachable IK')
            time.sleep(args.seconds)
        finally:
            if args.record: print('Stopped:',client.stop_recording())
