"""Move an ARI hand to an XYZ target with position-only IK and MoveIt planning."""
import argparse
from .client import ExperimentClient


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm',choices=['left','right'],default='right')
    p.add_argument('--xyz',type=float,nargs=3,required=True)
    p.add_argument('--frame',default='base_link')
    args=p.parse_args()
    with ExperimentClient(args.arm) as client:
        try:
            client.wait_ready(); print(client.move_xyz(*args.xyz,frame=args.frame))
        except KeyboardInterrupt: client.cancel()
