"""Verify exported timings, videos, measured state masks and action references."""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('exports',nargs='+',type=Path); args=parser.parse_args()
    for folder in args.exports:
        data=np.load(folder/'observations.npz',allow_pickle=False)
        times=data['stamp_ns']; assert len(times)>0 and np.all(np.diff(times)>0)
        metadata=json.loads((folder/'metadata.json').read_text())
        events=[json.loads(line) for line in (folder/'actions.jsonl').read_text().splitlines()]
        for key in ['position','velocity','effort']:
            assert np.array_equal(np.isfinite(data['joint_'+key]),data['joint_'+key+'_valid'])
        mask=data['joint_valid']; source=data['joint_source_stamp_ns']
        assert np.all(source[mask]<=times[mask]) and np.all(times[mask]-source[mask]<=100_000_000)
        assert np.all(source[~mask]==-1)
        for name in ['overhead','front','robot']:
            with (folder/f'{name}_frames.csv').open() as file: frames=list(csv.DictReader(file))
            capture=cv2.VideoCapture(str(folder/f'{name}.mp4')); count=0
            while True:
                good,image=capture.read()
                if not good: break
                count+=1
            capture.release()
            assert count==len(frames)==metadata['export']['camera_frames'][name]
            indices=data[name+'_frame_index']; valid=data[name+'_valid']
            original=np.array([int(frame['source_stamp_ns']) for frame in frames],dtype=np.int64)
            assert np.array_equal(data[name+'_source_stamp_ns'][valid],original[indices[valid]])
            assert np.all(original[indices[valid]]<=times[valid])
            assert np.all(times[valid]-original[indices[valid]]<=100_000_000)
            assert np.all(indices[~valid]==-1)
        valid=data['action_event_valid']; indices=data['action_event_index']
        stamps=np.array([event['stamp_ns'] for event in events],dtype=np.int64)
        assert np.all(stamps[indices[valid]]<=times[valid])
        modes={event['mode'] for event in events}
        if modes & {'joints','xyz'}:
            assert {'requested','planned','completed'}<={event['event'] for event in events}
        print('PASS:',folder,'observations=',len(times),'modes=',sorted(modes),flush=True)


if __name__=='__main__': main()
