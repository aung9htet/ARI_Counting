"""Keyboard joint jogging and scripted joint trajectories."""
import argparse
import math
import os
from pathlib import Path
import select
import sys
import termios
import threading
import tty
import yaml
from .client import ExperimentClient


def keyboard(client,step):
    if not sys.stdin.isatty(): raise RuntimeError('keyboard control requires an interactive terminal')
    limits=client.status()['limits'][client.arm]
    print('1–5 select joint; +/- jog; [/] halve/double step; c cancel/hold; q quit')
    for i,joint in enumerate(limits):
        print(f"{i+1}: {joint['name']} axis={joint['axis']} range={math.degrees(joint['lower']):.1f}..{math.degrees(joint['upper']):.1f} degrees")
    selected=0; pending=None
    saved=termios.tcgetattr(sys.stdin)
    def move(target):
        try: print('\r\n'+str(client.command_joints(target))+'\r')
        except Exception as exc: print('\r\n'+str(exc)+'\r')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if not select.select([sys.stdin],[],[],0.1)[0]: continue
            data=os.read(sys.stdin.fileno(),1)
            if not data: break  # terminal closed
            key=data.decode(errors='ignore')
            if not key: continue  # byte of a non-ASCII key; '' is "in" every string
            if key=='q': client.cancel(); break
            if key=='c': print('\r\n'+str(client.cancel())+'\r'); continue
            if key in '12345': selected=int(key)-1; print(f'\r\nSelected {selected+1}\r'); continue
            if key in '[]': step=max(0.001,min(0.2,step*(0.5 if key=='[' else 2))); continue
            if key not in '+-=': continue
            if pending and pending.is_alive(): print('\r\nMotion active; c to cancel\r'); continue
            observation=client.get_observation()
            state=observation['joint_state']
            if state is None: print('\r\nJoint data stale\r'); continue
            values=dict(zip(state.name,state.position)); targets=[values[j['name']] for j in limits]
            # Bring untouched joints inside their soft limits if startup is on a boundary.
            targets=[max(j['lower'],min(j['upper'],v)) for j,v in zip(limits,targets)]
            targets[selected]+=step*(-1 if key=='-' else 1)
            pending=threading.Thread(target=move,args=(targets,)); pending.start()
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,saved)
        client.cancel()
        if pending: pending.join(timeout=15)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm',choices=['left','right'],default='right')
    p.add_argument('--positions',type=float,nargs=5,help='five joint targets in radians')
    p.add_argument('--duration',type=float,default=2.0)
    p.add_argument('--trajectory',type=Path,help='YAML list of positions/time waypoints')
    p.add_argument('--step',type=float,default=0.05)
    args=p.parse_args()
    if args.positions is not None and args.trajectory is not None: p.error('choose positions or trajectory')
    with ExperimentClient(args.arm) as client:
        try:
            client.wait_ready()
            if args.positions is not None: print(client.command_joints(args.positions,args.duration))
            elif args.trajectory: print(client.command_trajectory(yaml.safe_load(args.trajectory.read_text())))
            else: keyboard(client,args.step)
        except KeyboardInterrupt: client.cancel()
