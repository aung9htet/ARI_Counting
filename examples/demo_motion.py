"""Clearly visible two-arm demonstration: record it, export it, write one playable video.

Run against a running simulator:
    scripts/ari run python3 /workspace/examples/demo_motion.py
"""
import argparse
from pathlib import Path
import subprocess
import time
from ari_counting_tools.client import ExperimentClient
from ari_counting_tools.export import export_episode

REST=[0.0,0.1,0.0,0.1,0.0]
# Raised-arm wave from PAL's ARI v2 motion library (ari_bringup, ari_v2_motions.yaml),
# slowed to stay inside the gateway's 50% joint-velocity rule. Times are absolute seconds.
WAVE=[{'positions':[-0.21,1.58,1.84,0.41,0.0],'time':3.5},
      {'positions':[-0.22,1.58,1.84,1.49,0.0],'time':5.5},
      {'positions':[-0.22,1.58,1.84,1.00,0.0],'time':6.7},
      {'positions':[-0.22,1.58,1.84,1.49,0.0],'time':7.9},
      {'positions':[-0.22,1.58,1.84,1.00,0.0],'time':9.1},
      {'positions':REST,'time':12.6}]
# Forward point toward the table. ARI's hand stops short of the table edge.
POINT=[{'positions':[0.9,0.15,0.0,0.2,0.0],'time':2.5},
       {'positions':[1.2,0.15,0.0,0.2,0.0],'time':4.0},
       {'positions':[0.9,0.15,0.0,0.2,0.0],'time':5.5},
       {'positions':REST,'time':8.0}]
# Hand targets in metres (world frame), ahead of the right shoulder and above the table height.
# IK starts from the measured posture: from arms-down it returns arbitrary valid elbow/wrist
# twists, so the arm is first raised to a posture near the targets for repeatable motion.
RAISED=[0.5,0.15,0.0,0.6,0.0]
REACH=[[0.22,-0.26,0.97],[0.18,-0.38,1.10],[0.26,-0.28,1.12]]


def move(robot):
    robot.arm='right'
    print('right arm wave:',robot.command_trajectory(WAVE),flush=True)
    robot.command_joints(RAISED,3)
    for target in REACH: print('right arm IK reach',target,robot.move_xyz(*target,frame='world'),flush=True)
    robot.command_joints(REST,5)
    robot.arm='left'
    print('left arm point:',robot.command_trajectory(POINT),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--no-record',action='store_true',help='only move, for watching in the Gazebo window')
    args=parser.parse_args()
    with ExperimentClient() as robot:
        robot.wait_ready()
        for robot.arm in ['left','right']: robot.command_joints(REST,5)
        if args.no_record: return move(robot)
        episode=robot.start_recording('Demonstration: wave, reach toward the table, point with the other arm.')
        try:
            move(robot); time.sleep(1)
        finally:
            robot.stop_recording()
    folder=Path(episode['episode'])/'export'
    print(export_episode(episode['episode']),flush=True)
    # The exporter writes MPEG-4 Part 2; browsers and editors need H.264.
    views=[str(folder/f'{name}.mp4') for name in ['front','overhead','robot']]
    subprocess.run(['ffmpeg','-y','-loglevel','error',*[arg for view in views for arg in ['-i',view]],
                    '-filter_complex','hstack=inputs=3','-c:v','libx264','-crf','20','-pix_fmt','yuv420p',
                    '-movflags','+faststart',str(folder/'demo.mp4')],check=True)
    print('Video (front | overhead | robot):',folder/'demo.mp4')


if __name__=='__main__': main()
