"""Start/stop persistent experiment recordings from any terminal."""
import argparse
from .client import ExperimentClient


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation',choices=['start','stop','status'])
    p.add_argument('--arm',choices=['left','right'],default='right')
    p.add_argument('--instruction',default='')
    args=p.parse_args()
    with ExperimentClient(args.arm) as client:
        if args.operation=='start':
            client.wait_ready(); print(client.start_recording(args.instruction))
        elif args.operation=='stop': print(client.stop_recording())
        else: print(client.status())
